"""
Pestañas Cinemáticas, Secuenciador y Cargador G-Code - CNC XYZW
Versión: 6.5 (Pure GRBL Engine)
Archivo: cnc_tabs.py
"""

import json
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QDoubleSpinBox, QSpinBox, QGroupBox, QComboBox, QFormLayout,
    QListWidget, QFileDialog, QMessageBox
)
from cnc_widgets import CalibrationDialog


class AxisTab(QWidget):
    def __init__(self, axis_name: str, main_window):
        super().__init__()
        self.axis = axis_name.lower()
        self.main_window = main_window

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(3)

        row_top = QHBoxLayout()
        row_top.setSpacing(3)
        self.btn_open_calibration = QPushButton(f"Calibración 2P ({self.axis.upper()})")
        self.btn_open_calibration.setToolTip("Abre el asistente interactivo de dos pasadas para corregir la resolución Steps/mm.")
        self.btn_invert_dir = QPushButton("Invertir Sentido Eje ($3)")
        self.btn_invert_dir.setStyleSheet("background-color: #8957e5; color: white;")
        row_top.addWidget(self.btn_open_calibration, 1)
        row_top.addWidget(self.btn_invert_dir, 1)
        lay.addLayout(row_top)

        gb_prof = QGroupBox("Perfil Curva en S & Aceleración ($$)")
        form_prof = QFormLayout(gb_prof)
        form_prof.setContentsMargins(6, 6, 6, 6)
        form_prof.setSpacing(3)

        self.sp_spm = QDoubleSpinBox()
        self.sp_spm.setRange(0.1, 100000.0)
        self.sp_spm.setDecimals(3)
        self.sp_spm.setValue(568.0)
        self.sp_spm.setFixedWidth(80)

        self.sp_max_rate = QDoubleSpinBox()
        self.sp_max_rate.setRange(1.0, 50000.0)
        self.sp_max_rate.setValue(1320.0)
        self.sp_max_rate.setFixedWidth(80)

        self.sp_accel = QDoubleSpinBox()
        self.sp_accel.setRange(1.0, 5000.0)
        self.sp_accel.setValue(30.0)
        self.sp_accel.setFixedWidth(80)

        self.sp_backlash = QDoubleSpinBox()
        self.sp_backlash.setRange(0.0, 5.0)
        self.sp_backlash.setDecimals(3)
        self.sp_backlash.setValue(0.0)
        self.sp_backlash.setFixedWidth(80)

        row_s1 = QHBoxLayout(); row_s1.setSpacing(4)
        row_s1.addWidget(self.sp_spm); row_s1.addWidget(QLabel("MaxRate:")); row_s1.addWidget(self.sp_max_rate); row_s1.addStretch(1)
        row_s2 = QHBoxLayout(); row_s2.setSpacing(4)
        row_s2.addWidget(self.sp_accel); row_s2.addWidget(QLabel("Backlash:")); row_s2.addWidget(self.sp_backlash); row_s2.addStretch(1)

        self.btn_set_profile = QPushButton("Guardar Cinemática en NVS ($)")
        self.btn_set_profile.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")

        form_prof.addRow("SPM/Rate ($10x/$11x):", row_s1)
        form_prof.addRow("Acel/BL ($12x/$4x):", row_s2)
        form_prof.addRow(self.btn_set_profile)
        lay.addWidget(gb_prof)

        gb_lim = QGroupBox("Límites y Homing ($)")
        form_lim = QFormLayout(gb_lim)
        form_lim.setContentsMargins(6, 6, 6, 6)
        form_lim.setSpacing(3)

        self.sp_max_travel = QDoubleSpinBox()
        self.sp_max_travel.setRange(1.0, 2000.0)
        self.sp_max_travel.setValue(110.0)
        self.sp_max_travel.setFixedWidth(85)

        self.sp_hseek = QDoubleSpinBox()
        self.sp_hseek.setRange(1.0, 5000.0)
        self.sp_hseek.setValue(500.0)
        self.sp_hseek.setFixedWidth(80)

        self.sp_hfeed = QDoubleSpinBox()
        self.sp_hfeed.setRange(1.0, 2000.0)
        self.sp_hfeed.setValue(50.0)
        self.sp_hfeed.setFixedWidth(80)

        self.sp_hpull = QDoubleSpinBox()
        self.sp_hpull.setRange(0.1, 50.0)
        self.sp_hpull.setValue(2.0)
        self.sp_hpull.setFixedWidth(85)

        row_hs = QHBoxLayout(); row_hs.setSpacing(4)
        row_hs.addWidget(self.sp_hseek); row_hs.addWidget(QLabel("Feed:")); row_hs.addWidget(self.sp_hfeed); row_hs.addStretch(1)

        form_lim.addRow("Carrera Max ($13x) [mm]:", self.sp_max_travel)
        form_lim.addRow("Seek/Feed ($25/$24):", row_hs)
        form_lim.addRow("Pulloff ($27) [mm]:", self.sp_hpull)

        self.btn_set_limits = QPushButton("Guardar Límites en NVS ($)")
        self.btn_set_limits.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")
        form_lim.addRow(self.btn_set_limits)
        lay.addWidget(gb_lim)

        lay.addStretch(1)

        self.btn_open_calibration.clicked.connect(self._open_calib)
        self.btn_invert_dir.clicked.connect(self._toggle_dir)
        self.btn_set_profile.clicked.connect(self._save_motion)
        self.btn_set_limits.clicked.connect(self._save_limits)

    def _open_calib(self):
        diag = CalibrationDialog(self.axis, self.main_window, self)
        if diag.exec():
            self.sp_spm.setValue(self.main_window.get_axis_spm(self.axis))

    def _toggle_dir(self):
        idx = ["x", "y", "z", "w"].index(self.axis)
        cur_mask = self.main_window.dir_invert_mask
        new_mask = cur_mask ^ (1 << idx)
        self.main_window.send_gcode(f"$3={new_mask}")
        self.main_window.dir_invert_mask = new_mask
        QMessageBox.information(self, "Sentido de Eje", f"Máscara $3 actualizada a {new_mask}.")

    def _save_motion(self):
        idx = ["x", "y", "z", "w"].index(self.axis)
        self.main_window.send_gcode(f"${100 + idx}={self.sp_spm.value():.3f}")
        self.main_window.send_gcode(f"${110 + idx}={self.sp_max_rate.value():.1f}")
        self.main_window.send_gcode(f"${120 + idx}={self.sp_accel.value():.1f}")
        self.main_window.send_gcode(f"${40 + idx}={self.sp_backlash.value():.3f}")
        QMessageBox.information(self, "NVS", f"Ajustes cinemáticos del eje {self.axis.upper()} guardados.")

    def _save_limits(self):
        idx = ["x", "y", "z", "w"].index(self.axis)
        self.main_window.send_gcode(f"${130 + idx}={self.sp_max_travel.value():.2f}")
        self.main_window.send_gcode(f"$25={self.sp_hseek.value():.1f}")
        self.main_window.send_gcode(f"$24={self.sp_hfeed.value():.1f}")
        self.main_window.send_gcode(f"$27={self.sp_hpull.value():.2f}")
        QMessageBox.information(self, "NVS", "Ajustes de límites y homing guardados en NVS.")


class SequenceProgrammerWidget(QWidget):
    def __init__(self, main_win, parent=None):
        super().__init__(parent)
        self.main_win = main_win
        self.sequence_queue = []
        self.current_step = 0
        self.is_running = False
        self.waiting_ack = False
        self.waiting_motion = False
        self._idle_seen = 0

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)

        gb = QGroupBox("Editor de Bloques Multieje (100% G-Code)")
        grid = QGridLayout(gb)
        grid.setContentsMargins(4, 4, 4, 4)
        grid.setSpacing(3)

        self.cb_act = QComboBox()
        self.cb_act.addItems([
            "Move Abs (Unitario)", "Move Multi Abs (4 Ejes)", "Move Rel (+)",
            "Move Rel (-)", "Pausa (s)", "Husillo ON (M3)", "Husillo OFF (M5)"
        ])
        self.cb_ax = QComboBox()
        self.cb_ax.addItems(["X", "Y", "Z", "W"])
        self.sp_val = QDoubleSpinBox()
        self.sp_val.setRange(-5000.0, 5000.0)
        self.sp_val.setValue(10.0)

        self.multi_box = QWidget()
        m_lay = QHBoxLayout(self.multi_box)
        m_lay.setContentsMargins(0, 0, 0, 0)
        self.sp_mx = QDoubleSpinBox(); self.sp_mx.setRange(-5000, 5000); self.sp_mx.setFixedWidth(50)
        self.sp_my = QDoubleSpinBox(); self.sp_my.setRange(-5000, 5000); self.sp_my.setFixedWidth(50)
        self.sp_mz = QDoubleSpinBox(); self.sp_mz.setRange(-5000, 5000); self.sp_mz.setFixedWidth(50)
        self.sp_mw = QDoubleSpinBox(); self.sp_mw.setRange(-5000, 5000); self.sp_mw.setFixedWidth(50)
        m_lay.addWidget(QLabel("X:")); m_lay.addWidget(self.sp_mx)
        m_lay.addWidget(QLabel("Y:")); m_lay.addWidget(self.sp_my)
        m_lay.addWidget(QLabel("Z:")); m_lay.addWidget(self.sp_mz)
        m_lay.addWidget(QLabel("W:")); m_lay.addWidget(self.sp_mw)
        self.multi_box.setVisible(False)

        self.btn_add = QPushButton("Insertar Orden")
        self.btn_add.setStyleSheet("background-color: #238636; color: white;")

        grid.addWidget(QLabel("Tipo:"), 0, 0)
        grid.addWidget(self.cb_act, 0, 1)
        grid.addWidget(self.btn_add, 0, 2)

        self.row_single = QWidget()
        rs = QHBoxLayout(self.row_single)
        rs.setContentsMargins(0, 0, 0, 0)
        rs.addWidget(QLabel("Eje:"))
        rs.addWidget(self.cb_ax)
        rs.addWidget(QLabel("Valor:"))
        rs.addWidget(self.sp_val)

        grid.addWidget(self.row_single, 1, 0, 1, 3)
        grid.addWidget(self.multi_box, 1, 0, 1, 3)
        lay.addWidget(gb)

        self.list_orders = QListWidget()
        self.list_orders.setStyleSheet("background-color: #0d1117; color: #00f0ff; font-family: Consolas;")
        lay.addWidget(self.list_orders, 1)

        r_file = QHBoxLayout()
        self.btn_up = QPushButton("▲ Subir")
        self.btn_down = QPushButton("▼ Bajar")
        self.btn_del = QPushButton("Eliminar")
        self.btn_clear = QPushButton("Limpiar")
        self.btn_open_seq = QPushButton("Abrir...")
        self.btn_save_seq = QPushButton("Guardar...")

        r_file.addWidget(self.btn_up)
        r_file.addWidget(self.btn_down)
        r_file.addWidget(self.btn_del)
        r_file.addWidget(self.btn_clear)
        r_file.addStretch(1)
        r_file.addWidget(self.btn_open_seq)
        r_file.addWidget(self.btn_save_seq)
        lay.addLayout(r_file)

        r_exec = QHBoxLayout()
        self.lbl_seq_status = QLabel("Listo.")
        self.lbl_seq_status.setStyleSheet("font-weight: bold; color: #7ee787;")
        self.btn_run = QPushButton("EJECUTAR PROGRAMA")
        self.btn_run.setStyleSheet("background-color: #1f6feb; color: white; font-weight: bold; padding: 6px;")

        self.btn_stop = QPushButton("DETENER (! + 0x18)")
        self.btn_stop.setStyleSheet("background-color: #a42424; color: white; font-weight: bold; padding: 6px;")

        r_exec.addWidget(self.lbl_seq_status, 1)
        r_exec.addWidget(self.btn_run)
        r_exec.addWidget(self.btn_stop)
        lay.addLayout(r_exec)

        self.cb_act.currentIndexChanged.connect(self._on_action_changed)
        self.btn_add.clicked.connect(self._add_cmd)
        self.btn_up.clicked.connect(self._move_up)
        self.btn_down.clicked.connect(self._move_down)
        self.btn_del.clicked.connect(lambda: self.list_orders.takeItem(self.list_orders.currentRow()))
        self.btn_clear.clicked.connect(self.list_orders.clear)
        self.btn_open_seq.clicked.connect(self._open_file)
        self.btn_save_seq.clicked.connect(self._save_file)
        self.btn_run.clicked.connect(self._start_seq)
        self.btn_stop.clicked.connect(self._stop_seq)

    def _on_action_changed(self, idx):
        is_multi = (idx == 1)
        is_spindle = (idx in (5, 6))
        self.multi_box.setVisible(is_multi)
        self.row_single.setVisible(not is_multi and not is_spindle)

    def _add_cmd(self):
        act = self.cb_act.currentText()
        if act == "Move Multi Abs (4 Ejes)":
            cmd = f"G90 G21 G1 X{self.sp_mx.value():.3f} Y{self.sp_my.value():.3f} Z{self.sp_mz.value():.3f} W{self.sp_mw.value():.3f} F600"
        elif act == "Move Abs (Unitario)":
            cmd = f"G90 G21 G1 {self.cb_ax.currentText()}{self.sp_val.value():.3f} F600"
        elif act == "Move Rel (+)":
            cmd = f"G91 G21 G1 {self.cb_ax.currentText()}{abs(self.sp_val.value()):.3f} F600"
        elif act == "Move Rel (-)":
            cmd = f"G91 G21 G1 {self.cb_ax.currentText()}{-abs(self.sp_val.value()):.3f} F600"
        elif act == "Pausa (s)":
            cmd = f"G4 P{self.sp_val.value():.2f}"
        elif act == "Husillo ON (M3)":
            cmd = "M3 S1000"
        else:
            cmd = "M5"
        self.list_orders.addItem(cmd)

    def _move_up(self):
        row = self.list_orders.currentRow()
        if row > 0:
            item = self.list_orders.takeItem(row)
            self.list_orders.insertItem(row - 1, item)
            self.list_orders.setCurrentRow(row - 1)

    def _move_down(self):
        row = self.list_orders.currentRow()
        if 0 <= row < self.list_orders.count() - 1:
            item = self.list_orders.takeItem(row)
            self.list_orders.insertItem(row + 1, item)
            self.list_orders.setCurrentRow(row + 1)

    def _open_file(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Abrir Secuencia", "", "Secuencia CNC (*.json *.txt);;Todos (*)")
        if not fn: return
        try:
            with open(fn, "r", encoding="utf-8") as f: data = json.load(f)
            self.list_orders.clear()
            for l in data: self.list_orders.addItem(str(l))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo abrir: {e}")

    def _save_file(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Guardar Secuencia", "", "Secuencia CNC (*.json);;Todos (*)")
        if not fn: return
        data = [self.list_orders.item(i).text() for i in range(self.list_orders.count())]
        try:
            with open(fn, "w", encoding="utf-8") as f: json.dump(data, f, indent=2)
            QMessageBox.information(self, "Guardado", "Secuencia guardada con éxito.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo guardar: {e}")

    def _start_seq(self):
        if self.list_orders.count() == 0 or not self.main_win.worker: return
        self.sequence_queue = [self.list_orders.item(i).text() for i in range(self.list_orders.count())]
        self.current_step = 0
        self.is_running = True
        self.waiting_ack = False
        self.waiting_motion = False
        self._idle_seen = 0
        self.lbl_seq_status.setText(f"Paso 1/{len(self.sequence_queue)}...")
        self._step_sequence()

    def _step_sequence(self):
        if not self.is_running: return
        if self.current_step >= len(self.sequence_queue):
            self.is_running = False
            self.lbl_seq_status.setText("Secuencia finalizada.")
            QMessageBox.information(self, "Secuenciador", "Secuencia completada exitosamente.")
            return

        cmd = self.sequence_queue[self.current_step]
        self.list_orders.setCurrentRow(self.current_step)
        self.lbl_seq_status.setText(f"Paso {self.current_step + 1}/{len(self.sequence_queue)}: {cmd}")

        self.waiting_ack = True
        self.waiting_motion = False
        self.main_win.send_gcode(cmd)

    def notify_ack(self, ok: bool):
        if self.is_running and self.waiting_ack:
            self.waiting_ack = False
            if ok:
                self.waiting_motion = True
                self._idle_seen = 0
            else:
                self.is_running = False
                self.lbl_seq_status.setText("Secuencia abortada por error.")
                QMessageBox.critical(self, "Secuenciador", "Orden rechazada por GRBL.")

    def notify_status(self, data: dict):
        if self.is_running and self.waiting_motion:
            st = data.get("state", "").lower()
            if st == "alarm":
                self.is_running = False
                self.waiting_motion = False
                self.lbl_seq_status.setText("Abortado: ALARM detectada.")
                QMessageBox.critical(self, "Secuenciador", "Alarma disparada. Secuencia cancelada.")
                return

            if st == "idle":
                self._idle_seen += 1
                if self._idle_seen >= 2:
                    self.waiting_motion = False
                    self.current_step += 1
                    self._step_sequence()
            else:
                self._idle_seen = 0

    def _stop_seq(self):
        self.is_running = False
        self.waiting_ack = False
        self.waiting_motion = False
        self.lbl_seq_status.setText("Secuencia detenida.")
        self.main_win.send_realtime(b"!")
        self.main_win.send_realtime(b"\x18")


class GCodeRunnerWidget(QWidget):
    def __init__(self, main_win, parent=None):
        super().__init__(parent)
        self.main_win = main_win
        self.lines = []
        self.idx = 0
        self.is_running = False
        self.waiting_ok = False
        self.is_paused = False

        self.watchdog = QTimer(self)
        self.watchdog.setSingleShot(True)
        self.watchdog.timeout.connect(self._on_timeout)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(4)

        row = QHBoxLayout()
        self.btn_open = QPushButton("Abrir G-code...")
        self.btn_run = QPushButton("EJECUTAR")
        self.btn_run.setStyleSheet("background-color: #1f6feb; color: white; font-weight: bold;")
        self.btn_pause = QPushButton("Pausar (!)")
        self.btn_stop = QPushButton("Cancelar (! + 0x18)")
        self.btn_stop.setStyleSheet("background-color: #a42424; color: white;")
        self.btn_run.setEnabled(False)
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)

        row.addWidget(self.btn_open)
        row.addWidget(self.btn_run)
        row.addWidget(self.btn_pause)
        row.addWidget(self.btn_stop)
        lay.addLayout(row)

        self.lbl_status = QLabel("Sin archivo cargado.")
        self.lbl_status.setStyleSheet("font-weight: bold; color: #7ee787;")
        lay.addWidget(self.lbl_status)

        self.list_lines = QListWidget()
        self.list_lines.setStyleSheet("background-color: #0d1117; font-family: Consolas; color: #d2a8ff;")
        lay.addWidget(self.list_lines, 1)

        self.btn_open.clicked.connect(self._open_file)
        self.btn_run.clicked.connect(self._start)
        self.btn_pause.clicked.connect(self._toggle_pause)
        self.btn_stop.clicked.connect(self._stop)

    @staticmethod
    def _clean_line(raw: str) -> str:
        out = []
        in_p = False
        for ch in raw:
            if in_p:
                if ch == ")": in_p = False
                continue
            if ch == "(": in_p = True; continue
            if ch == ";": break
            out.append(ch)
        return "".join(out).strip()

    def _open_file(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Abrir G-code", "", "G-Code (*.nc *.gcode *.tap *.txt);;Todos (*)")
        if not fn: return
        try:
            with open(fn, "r", encoding="utf-8", errors="replace") as f:
                raw_lines = f.read().splitlines()
        except Exception as e:
            QMessageBox.critical(self, "G-code", f"Error al abrir: {e}")
            return
        self.lines = []
        self.list_lines.clear()
        for raw in raw_lines:
            c = self._clean_line(raw)
            self.list_lines.addItem(raw)
            if c and c != "%":
                self.lines.append(c)
        self.idx = 0
        self.is_running = False
        self.is_paused = False
        self.btn_run.setEnabled(len(self.lines) > 0)
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText(f"{len(self.lines)} bloques ejecutables ({len(raw_lines)} totales).")

    def _start(self):
        if not self.lines or not self.main_win.worker: return
        self.is_running = True
        self.is_paused = False
        self.idx = 0
        self.btn_run.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_pause.setText("Pausar (!)")
        self.btn_stop.setEnabled(True)
        self._send_next()

    def _toggle_pause(self):
        if not self.is_running: return
        if not self.is_paused:
            self.is_paused = True
            self.main_win.send_realtime(b"!")
            self.btn_pause.setText("Reanudar (~)")
            self.lbl_status.setText("Mecanizado en Pausa (Hold).")
        else:
            self.is_paused = False
            self.main_win.send_realtime(b"~")
            self.btn_pause.setText("Pausar (!)")
            self.lbl_status.setText("Reanudando mecanizado...")

    def _send_next(self):
        if not self.is_running or self.is_paused: return
        if self.idx >= len(self.lines):
            self.is_running = False
            self.btn_run.setEnabled(True)
            self.btn_pause.setEnabled(False)
            self.btn_stop.setEnabled(False)
            self.lbl_status.setText("Archivo completado exitosamente.")
            QMessageBox.information(self, "G-code", "Programa completado exitosamente.")
            return

        line = self.lines[self.idx]
        self.lbl_status.setText(f"Bloque {self.idx + 1}/{len(self.lines)}: {line[:50]}")
        self.list_lines.setCurrentRow(self.idx)
        self.waiting_ok = True
        self.watchdog.start(180000)
        self.main_win.send_gcode(line)

    def notify_ack(self, ok: bool, detail: str):
        if not self.is_running or not self.waiting_ok: return
        self.watchdog.stop()
        self.waiting_ok = False
        if ok:
            self.idx += 1
            self._send_next()
        else:
            self.is_running = False
            self.btn_run.setEnabled(len(self.lines) > 0)
            self.btn_pause.setEnabled(False)
            self.btn_stop.setEnabled(False)
            QMessageBox.critical(self, "G-code", f"Error en línea {self.idx + 1}: {detail}")

    def _stop(self):
        self.is_running = False
        self.is_paused = False
        self.waiting_ok = False
        self.watchdog.stop()
        self.btn_run.setEnabled(len(self.lines) > 0)
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("Detenido.")
        self.main_win.send_realtime(b"!")
        self.main_win.send_realtime(b"\x18")

    def _on_timeout(self):
        if self.is_running:
            self._stop()
            QMessageBox.critical(self, "G-code", f"Timeout esperando ACK en línea {self.idx + 1}.")