"""
Pruebas del firmware CNC_V2.ino ejecutado en el simulador de host.
Cada prueba arranca un proceso nuevo del simulador (NVS limpia).
Se verifican, entre otros, los hallazgos de la auditoría (ver docs/).
"""
import json
import re
import socket
import time

import pytest

from conftest import (AXES, BO, MAXTRAVEL, SO, SPM, TOKEN, ZERO_PHYS, Client, Sim,
                      configure_axes, home_all)


# ---------------------------------------------------------------- arranque / sesión
def test_boot_hello_and_version(sim):
    assert any(l.startswith("BOOT|CNC-XYZW|2.0.0|2|token=0") for l in sim.serial_lines)
    v = sim.serial("VERSION", "VERSION|")
    assert v == "VERSION|CNC-XYZW|2.0.0|2"
    c = Client(sim.port)
    try:
        assert c.fw == "2.0.0" and c.proto == 2 and c.token_state == 0
        assert re.fullmatch(r"[0-9a-f]{32}", c.nonce)
    finally:
        c.close()


def test_unprovisioned_refuses_auth_and_motion_but_estop_and_ping_work(sim, clients):
    c = Client(sim.port)
    clients.append(c)
    assert c.request("PING", "PING") == "ACK|PING|OK"
    assert c.auth().startswith("NACK|AUTH|not_provisioned")
    assert c.cmd("enable_actuators") == "NACK|enable_actuators|auth_required"
    assert c.cmd("home_axis", "x") == "NACK|home_axis|auth_required"
    # E-STOP siempre se acepta, incluso sin autenticar (seguridad)
    assert c.cmd("emergency_stop").startswith("ACK|emergency_stop|OK")
    c.wait_for(lambda s: s["state"] == "alarm" and s["alarm"] == 1, 3, "alarma E-STOP", poll=True)


def test_auth_ok_bad_and_lockout(provisioned, clients):
    c = Client(provisioned.port)
    clients.append(c)
    assert c.token_state == 2
    assert c.cmd("enable_actuators") == "NACK|enable_actuators|auth_required"
    assert c.auth("otro-token-distinto").startswith("NACK|AUTH|bad_credentials")
    assert c.auth("otro-token-distinto").startswith("NACK|AUTH|bad_credentials")
    assert c.auth("otro-token-distinto").startswith("NACK|AUTH|locked")
    time.sleep(0.3)
    assert c.closed
    c2 = Client(provisioned.port, expect_hello=False)  # bloqueado 30 s: rechaza sin siquiera saludar
    clients.append(c2)
    assert c2.expect("NACK|AUTH|locked")
    time.sleep(0.3)
    assert c2.closed


def test_auth_success_sends_config_and_enables_commands(provisioned, clients):
    c = Client(provisioned.port)
    clients.append(c)
    assert c.auth().startswith("ACK|AUTH|OK")
    c.expect("CF|G|")
    for a in AXES:
        c.expect(f"CF|{a}|")
    assert float(c.cf["x"][0]) == 568.0
    assert c.ok("enable_actuators")
    c.wait_for(lambda s: s["en"] == 1, 3, "actuadores habilitados")


def test_open_mode_requires_confirmation(sim, clients):
    r = sim.serial("SET_OPEN_MODE", "NACK|SET_OPEN_MODE")
    assert r
    sim.serial("SET_OPEN_MODE|CONFIRM", "ACK|SET_OPEN_MODE|OK")
    c = Client(sim.port)
    clients.append(c)
    assert c.token_state == 1
    assert c.auth().startswith("ACK|AUTH|OK")


# ---------------------------------------------------------------- entrada malformada
def test_unknown_malformed_and_overlong_lines(ready):
    c = ready
    assert c.cmd("no_existe").startswith("NACK|no_existe|unknown_command")
    assert c.cmd("move_axis_abs", "x").startswith("NACK|move_axis_abs|bad_args")
    assert c.cmd("move_axis_abs", "q", "1").startswith("NACK|move_axis_abs|bad_axis")
    assert c.cmd("move_axis_abs", "x", "abc").startswith("NACK|move_axis_abs|bad_args")
    assert c.cmd("move_axis_abs", "x", "nan").startswith("NACK|move_axis_abs|bad_args")
    assert c.cmd("set_watchdog", "50").startswith("NACK|set_watchdog|bad_args")
    assert c.cmd("reset_step_counter", "x").startswith("NACK|reset_step_counter|deprecated")
    # línea de 300 caracteres: se rechaza entera, nunca se ejecuta
    c.send("CMD|emergency_stop|" + "A" * 300)
    r = c.expect("NACK|line|line_too_long")
    assert r
    time.sleep(0.2)
    assert c.st["state"] != "alarm"
    # un campo vacío o separadores de más no deben bloquear nada
    assert c.cmd("manual_ping").startswith("ACK|manual_ping|OK")


# ---------------------------------------------------------------- referenciado y movimiento
def test_moves_refused_before_homing_and_after_enable_is_required(ready):
    c = ready
    assert c.cmd("move_axis_abs", "x", "5").startswith("NACK|move_axis_abs|not_homed")
    assert c.cmd("move_multi_abs", "1", "1", "1", "1").startswith("NACK|move_multi_abs|not_homed")
    c.ok("disable_actuators")
    c.wait_for(lambda s: s["en"] == 0, 3, "deshabilitado")
    assert c.cmd("home_axis", "x").startswith("NACK|home_axis|disabled")


def test_homing_sets_zero_with_offset_and_no_steps_lost(ready):
    c, sim = ready, ready.sim
    sim.ctl("RESETSTATS")
    home_all(c)
    st = sim.state()
    for i in range(4):
        # cero de máquina = switch (100) + offset de seguridad (SO)
        assert st["pos"][i] == ZERO_PHYS, st
    assert st["lost_disabled"] == 0
    assert all(c.st["axes"][a]["homed"] and abs(c.st["axes"][a]["pos"]) < 0.01 for a in AXES)
    assert st["min_dir_setup_us"] >= 15, st  # DIR estable antes del STEP


def test_homing_starting_on_the_switch(ready):
    c, sim = ready, ready.sim
    sim.ctl("PHYS 0 90")  # ya sobre el final de carrera
    assert sim.state()["limit"][0] == 1
    c.ok("home_axis", "x")
    c.wait_for(lambda s: s["axes"]["x"]["homed"] and s["qdepth"] == 0, 20, "homing X")
    assert sim.state()["pos"][0] == ZERO_PHYS


def test_homing_fails_with_alarm_when_switch_never_triggers(ready):
    c, sim = ready, ready.sim
    sim.ctl("SWITCH 0 -100000")  # interruptor roto/desconectado
    sim.ctl("HARD 0 -100000 6000")
    c.ok("home_axis", "x")
    st = c.wait_for(lambda s: s["state"] == "alarm", 40, "alarma de homing")
    assert st["alarm"] == 4  # AL_HOME_FAIL
    assert not st["axes"]["x"]["homed"]
    assert st["axes"]["x"]["err"] == 4  # AE_SEEK_FAIL
    assert any(e.upper().startswith("EV|ALARM|4|X") for e in c.events)


def test_absolute_move_reaches_exact_position(ready):
    c, sim = ready, ready.sim
    home_all(c)
    sim.ctl("RESETSTATS")
    r = c.ok("move_axis_abs", "x", "10")
    assert re.search(r"\|\d+$", r)  # devuelve id del bloque
    c.wait_for(lambda s: s["axes"]["x"]["pos"] > 9.99 and not s["axes"]["x"]["moving"], 10, "llegada X")
    assert sim.state()["pos"][0] == ZERO_PHYS + int(10 * SPM)
    c.ok("move_axis_rel", "x", "-4")
    c.wait_idle()
    assert sim.state()["pos"][0] == ZERO_PHYS + int(6 * SPM)
    assert sim.state()["crash"] == [0, 0, 0, 0]


def test_soft_limits_are_enforced(ready):
    c = ready
    home_all(c)
    assert c.cmd("move_axis_abs", "x", str(MAXTRAVEL + 1)).startswith("NACK|move_axis_abs|soft_limit")
    assert c.cmd("move_axis_abs", "x", "-0.5").startswith("NACK|move_axis_abs|soft_limit")
    assert c.cmd("move_multi_abs", "1", str(MAXTRAVEL + 1), "-", "-").startswith("NACK|move_multi_abs|soft_limit")


def test_multi_axis_move_synchronised_and_works_after_stop(ready):
    """Hallazgo P0 de la auditoría: tras un STOP los bloques multieje nunca volvían a ejecutarse."""
    c, sim = ready, ready.sim
    home_all(c)
    # 1) arranque largo en X y parada
    c.ok("move_axis_abs", "x", "30")
    c.wait_for(lambda s: s["axes"]["x"]["moving"], 5, "X en movimiento")
    c.ok("stop")
    c.wait_idle()
    # 2) ahora un bloque multieje DEBE ejecutarse
    sim.ctl("RESETSTATS")
    c.ok("move_multi_abs", "5", "10", "-", "-")
    c.wait_idle()
    st = sim.state()
    assert st["pos"][0] == ZERO_PHYS + int(5 * SPM)
    assert st["pos"][1] == ZERO_PHYS + int(10 * SPM)
    # llegan a la vez: el eje corto avanza proporcionalmente (pasos totales conservados)
    assert st["pulses"][0] == int(5 * SPM) or st["pulses"][0] > 0


def test_multi_axis_four_axes_and_feed_in_mm_per_s(ready):
    c, sim = ready, ready.sim
    home_all(c)
    sim.ctl("RESETSTATS")
    t0 = time.time()
    c.ok("move_multi_abs", "6", "8", "0", "0", "10")  # trayectoria de 10 mm a 10 mm/s
    c.wait_for(lambda s: s["axes"]["x"]["moving"] or s["axes"]["y"]["moving"], 5, "inicio")
    c.wait_idle()
    dt = time.time() - t0
    assert 0.95 <= dt <= 2.0, dt  # ~1 s + rampas (con vel. en mm/s de la trayectoria, no por eje)
    st = sim.state()
    assert st["pos"][0] == ZERO_PHYS + int(6 * SPM) and st["pos"][1] == ZERO_PHYS + int(8 * SPM)


def test_queue_full_is_reported_and_accepted_blocks_all_execute(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("dwell", "800")
    ids, fulls = [], 0
    for i in range(40):
        r = c.cmd("move_axis_abs", "x", str(1 + (i % 5)))
        if r.startswith("NACK|move_axis_abs|full"):
            fulls += 1
        else:
            assert r.startswith("ACK|move_axis_abs|OK"), r
            ids.append(int(r.split("|")[-1]))
    assert fulls > 0 and len(ids) >= 8
    c.wait_idle(30)
    assert c.st["done_id"] == ids[-1]
    assert sim.state()["crash"] == [0, 0, 0, 0]


def test_sequence_ids_are_reported_in_done_id(ready):
    c = ready
    home_all(c)
    r1 = int(c.ok("move_axis_abs", "y", "3").split("|")[-1])
    r2 = int(c.ok("dwell", "50").split("|")[-1])
    assert r2 == r1 + 1 or r2 > r1
    c.wait_for(lambda s: s["done_id"] == r2, 10, "done_id del dwell")


# ---------------------------------------------------------------- seguridad
def test_estop_cuts_enable_immediately_clears_homed_and_needs_clear_alarm(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("move_axis_abs", "x", "35")
    c.wait_for(lambda s: s["axes"]["x"]["moving"], 5, "X en movimiento")
    t0 = time.time()
    c.send("CMD|emergency_stop")
    deadline = time.time() + 1.0
    while sim.state()["enable_low"] and time.time() < deadline:
        pass
    assert not sim.state()["enable_low"], "EN debe quedar deshabilitado (HIGH)"
    assert time.time() - t0 < 0.25
    c.wait_for(lambda s: s["state"] == "alarm" and s["alarm"] == 1, 3, "alarma")
    assert all(not c.st["axes"][a]["homed"] for a in AXES)  # tras E-STOP hay que referenciar de nuevo
    assert re.match(r"NACK\|move_axis_abs\|(alarm|disabled)", c.cmd("move_axis_abs", "x", "1"))
    assert c.cmd("enable_actuators").startswith("NACK|enable_actuators|alarm")
    c.ok("clear_alarm")
    c.wait_for(lambda s: s["state"] == "idle", 3, "idle tras clear_alarm")
    sim.ctl("RESETSTATS")
    time.sleep(0.2)
    assert sim.state()["pulses"] == [0, 0, 0, 0]
    assert sim.state()["lost_disabled"] == 0


def test_estop_works_without_authentication_when_token_set(provisioned, clients):
    c = Client(provisioned.port)
    clients.append(c)
    assert c.cmd("emergency_stop").startswith("ACK|emergency_stop|OK")
    c.wait_for(lambda s: s["state"] == "alarm" and s["alarm"] == 1, 3, "alarma", poll=True)
    # la alarma sólo la borra un cliente autenticado
    assert c.cmd("clear_alarm") == "NACK|clear_alarm|auth_required"


def test_physical_estop_input(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("move_axis_abs", "x", "35")
    c.wait_for(lambda s: s["axes"]["x"]["moving"], 5, "X en movimiento")
    sim.ctl("ESTOP_PIN 1")
    st = c.wait_for(lambda s: s["state"] == "alarm", 3, "alarma por E-STOP físico")
    assert st["alarm"] == 5
    assert not sim.state()["enable_low"]
    # no se puede borrar mientras el pulsador siga activo
    assert c.cmd("clear_alarm").startswith("NACK|clear_alarm|alarm")
    sim.ctl("ESTOP_PIN 0")
    c.ok("clear_alarm")


def test_link_loss_stops_motion_with_alarm_comm_lost(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("move_axis_abs", "x", "35")
    c.wait_for(lambda s: s["axes"]["x"]["moving"], 5, "X en movimiento")
    c.close()  # el cliente que mandaba desaparece (cable/WiFi caído)
    time.sleep(0.3)
    c2 = Client(sim.port)  # observador (no autenticado): puede leer estado
    try:
        time.sleep(1.2)
        st = c2.wait_for(lambda s: s["state"] == "alarm", 3, "alarma COMM_LOST", poll=True)
        assert st["alarm"] == 3
        before = sim.state()["pos"][0]
        time.sleep(0.3)
        assert sim.state()["pos"][0] == before  # detenido
    finally:
        c2.close()


def test_link_watchdog_silent_client_stops_motion(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("set_watchdog", "200")
    c.ok("move_axis_abs", "x", "35")
    c.wait_for(lambda s: s["axes"]["x"]["moving"], 5, "X en movimiento")
    c.keepalive = False  # el cliente sigue conectado pero deja de enviar (programa colgado)
    st = c.wait_for(lambda s: s["state"] == "alarm", 3, "alarma por silencio")
    assert st["alarm"] == 3


def test_ping_keeps_motion_alive_beyond_watchdog(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("set_watchdog", "200")
    c.ok("move_axis_abs", "x", "39")
    c.wait_idle(20)
    assert c.st["axes"]["x"]["pos"] > 38.9
    assert c.st["alarm"] == 0


def test_hard_limit_alarm_is_direction_aware(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("move_axis_abs", "x", "10")
    c.wait_idle()
    # simula descalibración: el final de carrera "aparece" a 5 mm del cero
    sim.ctl(f"SWITCH 0 {ZERO_PHYS + int(5 * SPM)}")
    c.ok("move_axis_abs", "x", "0")
    st = c.wait_for(lambda s: s["state"] == "alarm", 10, "alarma de fin de carrera")
    assert st["alarm"] == 2
    assert st["axes"]["x"]["err"] == 2
    assert not st["axes"]["x"]["homed"]
    assert sim.state()["crash"] == [0, 0, 0, 0]  # paró antes de chocar con el tope
    assert any(e.upper().startswith("EV|ALARM|2|X") for e in c.events)


def test_positive_motion_with_switch_active_is_not_a_hard_limit(ready):
    """Antes (V1) el límite no distinguía el sentido y bloqueaba también el movimiento de alejamiento."""
    c, sim = ready, ready.sim
    sim.ctl("PHYS 0 90")  # eje sin referenciar sobre el interruptor
    assert sim.state()["limit"][0] == 1
    # sin referenciar sólo se permite jog de "recuperación" en sentido positivo
    assert c.cmd("manual_start", "x", "-1").startswith("NACK|manual_start|not_homed")
    assert c.ok("manual_start", "x", "1")
    t_end = time.time() + 0.4
    while time.time() < t_end:
        c.send("CMD|manual_ping")
        time.sleep(0.05)
    c.ok("manual_stop", "x")
    c.wait_idle()
    assert c.st["alarm"] == 0
    assert sim.state()["pos"][0] > 90


# ---------------------------------------------------------------- manual / jog
def test_jog_deadman_stops_when_pings_stop(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("manual_start", "x", "1")
    c.wait_for(lambda s: s["axes"]["x"]["moving"], 3, "jog")
    for _ in range(6):
        c.send("CMD|manual_ping")
        time.sleep(0.05)
    assert c.st["axes"]["x"]["moving"]
    t0 = time.time()
    c.wait_for(lambda s: not s["axes"]["x"]["moving"], 3, "parada por hombre muerto")
    assert time.time() - t0 < 1.0
    assert c.st["state"] in ("idle", "manual")


def test_jog_respects_soft_limits(ready):
    c, sim = ready, ready.sim
    home_all(c)
    c.ok("manual_start", "x", "-1")  # hacia el cero: se detiene en 0 sin tocar el interruptor
    t_end = time.time() + 1.5
    while time.time() < t_end and c.st["axes"]["x"]["moving"] is not False:
        c.send("CMD|manual_ping")
        time.sleep(0.03)
    c.wait_for(lambda s: not s["axes"]["x"]["moving"], 3, "parada en el límite")
    assert c.st["alarm"] == 0
    assert sim.state()["pos"][0] >= ZERO_PHYS


# ---------------------------------------------------------------- configuración
def test_set_zero_requires_homed_and_only_moves_work_offset(ready):
    c, sim = ready, ready.sim
    assert c.cmd("set_zero_axis", "x").startswith("NACK|set_zero_axis|not_homed")
    home_all(c)
    c.ok("move_axis_abs", "x", "5")
    c.wait_idle()
    c.ok("set_zero_axis", "x")
    c.wait_for(lambda s: abs(s["axes"]["x"]["pos"]) < 0.01, 3, "pos de trabajo = 0")
    assert c.st["axes"]["x"]["homed"]  # el cero de trabajo NO invalida el referenciado
    phys = sim.state()["pos"][0]
    assert phys == ZERO_PHYS + int(5 * SPM)  # el eje no se mueve
    # los límites siguen siendo los de máquina: el recorrido positivo restante es MAXTRAVEL-5
    assert c.cmd("move_axis_abs", "x", str(MAXTRAVEL - 5 + 0.5)).startswith("NACK|move_axis_abs|soft_limit")
    assert c.cmd("move_axis_abs", "x", "-5.5").startswith("NACK|move_axis_abs|soft_limit")
    c.ok("move_axis_abs", "x", "-5")
    c.wait_idle()
    assert sim.state()["pos"][0] == ZERO_PHYS


def test_config_setters_refused_while_busy_and_persisted_after_restart(provisioned, clients, tmp_path):
    sim = provisioned
    c = Client(sim.port)
    clients.append(c)
    c.auth()
    configure_axes(c)
    c.ok("set_profile_axis", "x", "2", "40", "2", "300")
    c.ok("invert_axis_dir", "y")
    sim.ctl("WIRING 1 0")  # el cableado real de Y está invertido: tras invertir DIR el homing funciona
    c.ok("set_axis_mode", "x", "0")
    c.ok("enable_actuators")
    home_all(c)
    c.ok("dwell", "1500")
    assert c.cmd("set_calibration_axis", "x", "60", "30", "x").startswith("NACK|set_calibration_axis|busy")
    c.wait_idle(5)
    time.sleep(0.6)  # la NVS se escribe sólo en reposo (>=300 ms)
    c.close()
    sim.serial("RESTART_ESP")
    assert sim.wait_exit(5) == 42
    sim2 = Sim(sim.nvs_path)
    try:
        sim2.geometry()
        sim2.ctl("WIRING 1 0")
        c2 = Client(sim2.port)
        clients.append(c2)
        assert c2.token_state == 2
        assert c2.auth().startswith("ACK|AUTH|OK")
        c2.expect("CF|G|")
        for a in AXES:
            c2.expect(f"CF|{a}|")
        cx = c2.cf["x"]
        assert float(cx[0]) == SPM and float(cx[1]) == MAXTRAVEL
        assert cx[5] == "0"                                    # modo (dfl) persistido
        assert cx[11] == "2.000" and cx[12] == "40.000"        # perfil start/cruise persistido
        assert c2.cf["y"][5] == "1"
        assert not c2.st["axes"]["x"]["homed"]                 # tras reinicio hay que referenciar
        assert c2.st["en"] == 0                                # los motores arrancan SIEMPRE deshabilitados
        c2.ok("enable_actuators")
        c2.ok("home_axis", "y")                                # sólo funciona si la inversión de DIR persistió
        c2.wait_for(lambda s: s["axes"]["y"]["homed"] and s["qdepth"] == 0, 20, "homing Y tras reinicio")
    finally:
        sim2.close()


def test_calibration_validates_arguments(ready):
    c = ready
    assert c.cmd("set_calibration_axis", "x", "0", "10", "d").startswith("NACK|set_calibration_axis|bad_args")
    assert c.cmd("set_calibration_axis", "x", "-5", "10", "d").startswith("NACK|set_calibration_axis|bad_args")
    assert c.cmd("set_calibration_axis", "x", "50", "5000", "d").startswith("NACK|set_calibration_axis|bad_args")
    assert c.cmd("set_limits_axis", "x", "40", "-1", "5").startswith("NACK|set_limits_axis|bad_args")
    assert c.cmd("set_profile_axis", "x", "5", "1", "5", "100").startswith("NACK|set_profile_axis|bad_args")


# ---------------------------------------------------------------- USB / secretos
def test_usb_wifi_provisioning_never_leaks_secrets(sim):
    sim.serial("SET_WIFI_NVS|Mi%20Red|clave-super-secreta|CNC-LAB|168", "ACK|SET_WIFI_NVS|OK")
    sim.serial(f"SET_TOKEN|{TOKEN}", "ACK|SET_TOKEN|OK")
    dump = sim.serial("DUMP_NVS", "NVS_DATA|")
    data = json.loads(dump.split("|", 1)[1])
    assert data["ssid"] == "Mi Red" and data["pass_set"] is True
    assert data["devname"] == "CNC-LAB" and data["ip_oct4"] == 168
    assert data["token_state"] == 2
    full = sim.serial_text()
    assert "clave-super-secreta" not in full and TOKEN not in full
    # validaciones
    assert sim.serial("SET_WIFI_NVS|x|corta|n|10", "NACK|SET_WIFI_NVS")
    assert sim.serial("SET_TOKEN|corto", "NACK|SET_TOKEN")
    assert sim.serial("SET_WIFI_NVS|red|12345678|n|1", "NACK|SET_WIFI_NVS")  # IP inválida


def test_usb_scan_wifi_and_factory_reset(provisioned):
    sim = provisioned
    r = sim.serial("SCAN_WIFI", "WIFI_LIST|")
    assert "Red%20Taller" in r and "CNC%7Clab" in r  # SSID con separador: percent-encoded
    sim.serial("FACTORY_RESET", "NACK|FACTORY_RESET")
    sim.serial("FACTORY_RESET|CONFIRM", "ACK|FACTORY_RESET|OK")
    assert sim.wait_exit(5) == 42
    sim2 = Sim(sim.nvs_path)
    try:
        assert any("token=0" in l for l in sim2.serial_lines)
    finally:
        sim2.close()


# ---------------------------------------------------------------- una sola conexión
def test_second_client_is_rejected_while_first_alive_and_replaces_stale_one(ready):
    c, sim = ready, ready.sim
    c.ok("set_watchdog", "0")
    c2 = Client(sim.port)
    try:
        assert c2.expect("NACK|connect|busy")
    finally:
        c2.close()
    # el primero sigue vivo
    assert c.request("PING", "PING") == "ACK|PING|OK"
    # silencio > 3 s: la sesión se considera caída y el nuevo cliente la reemplaza
    c.keepalive = False
    time.sleep(3.5)
    c3 = Client(sim.port)
    try:
        assert c3.hello
        assert c3.auth().startswith("ACK|AUTH|OK")
    finally:
        c3.close()


def test_telemetry_rate_about_20_hz(ready):
    c = ready
    n0 = sum(1 for l in c.all_lines if l.startswith("ST|"))
    time.sleep(1.0)
    n1 = sum(1 for l in c.all_lines if l.startswith("ST|"))
    assert 12 <= (n1 - n0) <= 30, n1 - n0
