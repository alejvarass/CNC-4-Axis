"""
Infraestructura de pruebas del firmware CNC_V2.ino sobre el simulador de host.

El simulador (tests/sim/sim_main.cpp) compila el .ino SIN modificarlo y expone:
  - el TCP real del firmware (puerto SIM_PORT),
  - un canal de control por stdin/stdout (geometría de la máquina virtual,
    final de carrera, topes, E-STOP físico, puerto serie USB).
"""
import hashlib
import hmac
import json
import os
import queue
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SIM_BIN = Path(os.environ.get("CNC_SIM_BIN", "/tmp/cnc_sim/cnc_sim"))
TOKEN = "token-de-prueba-123"

AXES = "xyzw"


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(scope="session", autouse=True)
def _build_sim():
    """Compila el simulador si no existe o si el firmware cambió."""
    src = [ROOT / "CNC_V2.ino", ROOT / "tests/sim/sim_main.cpp", ROOT / "tests/sim/mock/Arduino.h"]
    if SIM_BIN.exists() and all(SIM_BIN.stat().st_mtime >= p.stat().st_mtime for p in src):
        return
    r = subprocess.run([str(ROOT / "tests/sim/build.sh"), str(SIM_BIN)], capture_output=True, text=True)
    if r.returncode != 0:
        pytest.fail("No compila el simulador:\n" + r.stdout + r.stderr)


class Sim:
    """Proceso del simulador + canal de control."""

    def __init__(self, nvs_path, port=None):
        self.port = port or _free_port()
        self.nvs_path = nvs_path
        env = dict(os.environ, SIM_PORT=str(self.port), SIM_NVS=str(nvs_path),
                   ASAN_OPTIONS="detect_leaks=0:abort_on_error=0",
                   UBSAN_OPTIONS="print_stacktrace=1")
        self.p = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True, env=env, bufsize=1)
        self.ctl_q = queue.Queue()
        self.serial_lines = []
        self.serial_cv = threading.Condition()
        self.stderr_text = []
        threading.Thread(target=self._read_out, daemon=True).start()
        threading.Thread(target=self._read_err, daemon=True).start()
        first = self.ctl_q.get(timeout=10)
        assert first == "READY", first

    def _read_out(self):
        for line in self.p.stdout:
            line = line.rstrip("\n")
            if line.startswith("CTL|"):
                self.ctl_q.put(line[4:])
            elif line.startswith("S|"):
                with self.serial_cv:
                    self.serial_lines.append(line[2:])
                    self.serial_cv.notify_all()

    def _read_err(self):
        for line in self.p.stderr:
            self.stderr_text.append(line)

    def ctl(self, line, timeout=5):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        return self.ctl_q.get(timeout=timeout)

    def state(self):
        return json.loads(self.ctl("GET"))

    def geometry(self, start=400, switch=100, hard_min=0, hard_max=6000):
        for a in range(4):
            self.ctl(f"PHYS {a} {start}")
            self.ctl(f"SWITCH {a} {switch}")
            self.ctl(f"HARD {a} {hard_min} {hard_max}")

    def serial(self, cmd, expect_prefix=None, timeout=5):
        """Envía una línea por el puerto serie USB y espera una respuesta que empiece por expect_prefix."""
        with self.serial_cv:
            start = len(self.serial_lines)
        self.ctl("SERIAL " + cmd)
        if expect_prefix is None:
            return None
        deadline = time.time() + timeout
        with self.serial_cv:
            while True:
                for ln in self.serial_lines[start:]:
                    if ln.startswith(expect_prefix):
                        return ln
                left = deadline - time.time()
                if left <= 0:
                    raise TimeoutError(f"sin respuesta serie {expect_prefix!r}; recibido: {self.serial_lines[start:]}")
                self.serial_cv.wait(left)

    def serial_text(self):
        with self.serial_cv:
            return "\n".join(self.serial_lines)

    def wait_exit(self, timeout=5):
        return self.p.wait(timeout=timeout)

    def close(self):
        try:
            if self.p.poll() is None:
                self.p.stdin.write("QUIT\n")
                self.p.stdin.flush()
                self.p.wait(timeout=3)
        except Exception:
            self.p.kill()
        if self.p.poll() is None:
            self.p.kill()

    def sanitizer_errors(self):
        txt = "".join(self.stderr_text)
        return txt if ("AddressSanitizer" in txt or "runtime error" in txt) else ""


class Client:
    """Cliente NET-LOG de prueba (mismo protocolo que la GUI)."""

    def __init__(self, port, timeout=5, expect_hello=True):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.sock.settimeout(0.2)
        self.q = queue.Queue()
        self.all_lines = []
        self.st = None
        self.st_count = 0
        self.cf = {}
        self.events = []
        self.closed = False
        self._buf = b""
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stop = False
        self.keepalive = True   # igual que la GUI: PING cada 50 ms desde un hilo propio
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._pinger, daemon=True).start()
        self.hello = None
        if expect_hello:
            self.hello = self.expect("HELLO|")
            parts = self.hello.split("|")
            self.fw, self.proto, self.token_state, self.nonce = parts[2], int(parts[3]), int(parts[4]), parts[5]

    def _reader(self):
        while not self._stop:
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                self.closed = True
                break
            self._buf += data
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                self._handle(line.decode(errors="replace").rstrip("\r"))
        self.closed = True

    def _pinger(self):
        while not self._stop and not self.closed:
            if self.keepalive:
                try:
                    self.send("PING")
                except OSError:
                    return
            time.sleep(0.05)

    def _handle(self, line):
        with self._lock:
            self.all_lines.append(line)
        if line.startswith("ST|"):
            self.st = self.parse_st(line)
            self.st_count += 1
            return
        if line.startswith("CF|"):
            f = line.split("|")
            self.cf[f[1]] = f[2:]
        if line.startswith("EV|"):
            self.events.append(line)
        self.q.put(line)

    @staticmethod
    def parse_st(line):
        f = line.split("|")
        st = {"state": f[1], "en": int(f[2]), "alarm": int(f[3]), "qdepth": int(f[4]), "done_id": int(f[5]), "axes": {}}
        for a, raw in zip(AXES, f[6:10]):
            pos, tgt, steps, flags, d, err = raw.split(",")
            flags = int(flags)
            st["axes"][a] = {"pos": float(pos), "target": float(tgt), "steps": int(steps), "flags": flags,
                             "homed": bool(flags & 1), "calibrated": bool(flags & 2), "moving": bool(flags & 4),
                             "limit": bool(flags & 8), "dir": int(d), "err": int(err)}
        return st

    def send(self, line):
        with self._send_lock:
            self.sock.sendall((line + "\n").encode())

    def expect(self, prefix, timeout=5):
        deadline = time.time() + timeout
        while True:
            left = deadline - time.time()
            if left <= 0:
                raise TimeoutError(f"no llegó {prefix!r}; recibido: {self.all_lines[-8:]}")
            try:
                line = self.q.get(timeout=left)
            except queue.Empty:
                continue
            if line.startswith(prefix):
                return line

    def request(self, line, key, timeout=5):
        """Envía la línea y devuelve el primer ACK/NACK de 'key'."""
        self.send(line)
        deadline = time.time() + timeout
        while True:
            left = deadline - time.time()
            if left <= 0:
                raise TimeoutError(f"sin respuesta a {line!r}; recibido: {self.all_lines[-8:]}")
            try:
                r = self.q.get(timeout=left)
            except queue.Empty:
                continue
            f = r.split("|")
            if f[0] in ("ACK", "NACK") and len(f) > 1 and f[1] == key:
                return r

    def cmd(self, name, *args, timeout=5):
        line = "CMD|" + name + "".join("|" + str(a) for a in args)
        return self.request(line, name, timeout)

    def ok(self, name, *args):
        r = self.cmd(name, *args)
        assert r.startswith(f"ACK|{name}|OK"), r
        return r

    def auth(self, token=TOKEN):
        mac = hmac.new(token.encode(), self.nonce.encode(), hashlib.sha256).hexdigest()
        return self.request("AUTH|" + mac, "AUTH")

    def wait_for(self, pred, timeout=10, what="condición", poll=False):
        """Espera a que la telemetría cumpla 'pred'. poll=True: cliente sin autenticar (usa GET_STATUS)."""
        deadline = time.time() + timeout
        next_poll = 0
        while time.time() < deadline:
            if poll and time.time() >= next_poll:
                self.send("GET_STATUS")
                next_poll = time.time() + 0.05
            st = self.st
            if st is not None and pred(st):
                return st
            time.sleep(0.005)
        raise TimeoutError(f"timeout esperando {what}; último ST: {self.st}")

    def wait_idle(self, timeout=15):
        """Espera reposo con la cola vacía, usando sólo telemetría posterior a la llamada (>=2 muestras nuevas)."""
        n0 = self.st_count
        deadline = time.time() + timeout
        while time.time() < deadline:
            st = self.st
            if self.st_count >= n0 + 2 and st["state"] == "idle" and st["qdepth"] == 0:
                return st
            time.sleep(0.005)
        raise TimeoutError(f"timeout esperando reposo; último ST: {self.st}")

    def close(self):
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass


# Geometría y perfil de prueba: pocos pasos y velocidades altas => ensayos de pocos segundos
SPM = 50.0           # pasos/mm
MAXTRAVEL = 40.0     # mm
BO = 50              # pasos de retroceso (1 mm)
SO = 100             # pasos de offset (2 mm)
ZERO_PHYS = 100 + SO  # posición física del cero de máquina tras HOME (switch en 100)


def configure_axes(c, axes=AXES, spm=SPM, maxt=MAXTRAVEL):
    for a in axes:
        c.ok("set_calibration_axis", a, spm, maxt, "2026-01-01")
        c.ok("set_limits_axis", a, maxt, BO, SO)
        c.ok("set_homing_speed_axis", a, 150, 300, 200)
        c.ok("set_manual_speed_axis", a, 300, 300)
        c.ok("set_profile_axis", a, 3, 60, 3, 600)


@pytest.fixture
def sim(tmp_path):
    s = Sim(tmp_path / "nvs.txt")
    s.geometry()
    yield s
    s.close()
    err = s.sanitizer_errors()
    assert not err, "Sanitizer del simulador:\n" + err


@pytest.fixture
def provisioned(sim):
    """Simulador con token instalado por USB."""
    r = sim.serial(f"SET_TOKEN|{TOKEN}", "ACK|SET_TOKEN|OK")
    assert r
    return sim


@pytest.fixture
def clients():
    cs = []
    yield cs
    for c in cs:
        c.close()


@pytest.fixture
def ready(provisioned, clients):
    """Cliente autenticado, ejes configurados, actuadores habilitados."""
    c = Client(provisioned.port)
    clients.append(c)
    r = c.auth()
    assert r.startswith("ACK|AUTH|OK"), r
    configure_axes(c)
    c.ok("enable_actuators")
    c.wait_for(lambda s: s["en"] == 1, 3, "enable")
    c.sim = provisioned
    return c


def home_all(c, timeout=40):
    for a in AXES:
        c.ok("home_axis", a)
    c.wait_for(lambda s: all(s["axes"][a]["homed"] for a in AXES) and s["qdepth"] == 0, timeout, "homing de los 4 ejes")
