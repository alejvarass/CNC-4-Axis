"""
Módulo de Comunicación y Protocolo GRBL v1.1 - CNC XYZW
Versión: 6.5 (Pure GRBL Engine)
Archivo: grbl_driver.py
"""

import socket
import logging
from PySide6.QtCore import QThread, Signal

log = logging.getLogger("GRBL_Driver")


class GrblTcpWorker(QThread):
    connected = Signal()
    disconnected = Signal(str)
    status_received = Signal(dict)
    ack_received = Signal(bool, str)
    message_received = Signal(str)

    def __init__(self):
        super().__init__()
        self.host = "192.168.1.167"
        self.port = 5000
        self.sock = None
        self.running = False
        self._rx_buf = b""

    def configure(self, host: str, port: int = 5000):
        self.host = host
        self.port = port

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.5)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.05)
            self.running = True
            self.connected.emit()

            while self.running:
                try:
                    chunk = self.sock.recv(2048)
                    if not chunk:
                        self.running = False
                        self.disconnected.emit("El servidor cerró la conexión TCP.")
                        break
                    self._rx_buf += chunk
                    if len(self._rx_buf) > 65536:
                        self._rx_buf = self._rx_buf[-4096:]
                    while b"\n" in self._rx_buf:
                        line, self._rx_buf = self._rx_buf.split(b"\n", 1)
                        s = line.decode("utf-8", errors="replace").strip()
                        if s:
                            self._parse_line(s)
                except socket.timeout:
                    continue
                except Exception as e:
                    self.running = False
                    self.disconnected.emit(str(e))
                    break
        except Exception as e:
            self.running = False
            self.disconnected.emit(str(e))
        finally:
            self._close_socket()

    def _parse_line(self, s: str):
        if s.startswith("<") and s.endswith(">"):
            raw = s[1:-1]
            parts = raw.split("|")
            d = {
                "state": parts[0],
                "mpos": [0.0, 0.0, 0.0, 0.0],
                "wpos": [0.0, 0.0, 0.0, 0.0],
                "fs": [0.0, 0.0],
                "pn": ""
            }
            for p in parts[1:]:
                if p.startswith("MPos:"):
                    d["mpos"] = [float(x) for x in p[5:].split(",")]
                elif p.startswith("WPos:"):
                    d["wpos"] = [float(x) for x in p[5:].split(",")]
                elif p.startswith("FS:"):
                    d["fs"] = [float(x) for x in p[3:].split(",")]
                elif p.startswith("Pn:"):
                    d["pn"] = p[3:]
            self.status_received.emit(d)
        elif s == "ok":
            self.ack_received.emit(True, "ok")
        elif s.startswith("error:"):
            self.ack_received.emit(False, s)
        else:
            self.message_received.emit(s)

    def send_line(self, line: str):
        if self.sock and self.running:
            try:
                self.sock.settimeout(1.0)
                self.sock.sendall((line.strip() + "\r\n").encode("utf-8"))
                self.sock.settimeout(0.05)
            except Exception as e:
                self.running = False
                self.disconnected.emit(str(e))

    def send_realtime(self, ch):
        if self.sock and self.running:
            try:
                if isinstance(ch, (bytes, bytearray)):
                    payload = bytes(ch)
                elif isinstance(ch, int):
                    payload = bytes([ch])
                else:
                    payload = bytes([ord(c) for c in ch])
                self.sock.sendall(payload)
                return True
            except Exception as e:
                log.error("Error en send_realtime: %s", e)
                return False
        return False

    def stop(self):
        self.running = False
        self.wait(200)

    def _close_socket(self):
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception:
                pass
            self.sock = None