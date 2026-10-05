"""
Herramienta de Diagnóstico, Mantenimiento y Configuración NVS - CNC XYZW
Comunicación: Puerto Serie UART (115200 Baud)
Versión: 6.5.1
Archivo: maintenance_tool.py
"""

import sys
import serial
import serial.tools.list_ports
from PySide6.QtCore import Qt, QThread, Signal, QTimer
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QGroupBox, QMessageBox, QComboBox, QCheckBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QDoubleSpinBox, QSpinBox
)
from cnc_widgets import DPadPainter, VerticalZWControl


class SerialWorker(QThread):
    line_received = Signal(str)
    connected = Signal()
    disconnected = Signal(str)

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            self.connected.emit()
            while self.running:
                try:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self.line_received.emit(line)
                except Exception as e:
                    self.running = False
                    self.disconnected.emit(str(e))
                    break
        except Exception as e:
            self.disconnected.emit(str(e))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def send_line(self, line: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((line.strip() + "\r\n").encode("utf-8"))
            except Exception:
                pass

    def send_realtime(self, b_char):
        if self.ser and self.ser.is_open:
            try:
                if isinstance(b_char, (bytes, bytearray)):
                    self.ser.write(b_char)
                elif isinstance(b_char, int):
                    self.ser.write(bytes([b_char]))
                else:
                    self.ser.write(bytes([ord(b_char)]))
            except Exception:
                pass

    def stop(self):
        self.running = False
        self.wait(300)


class MaintenanceMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mantenimiento y Configuración NVS CNC XYZW (Puerto Serie)")
        self.resize(1100, 720)

        self.worker = None
        self.active_axis_mask = 15  # 0b1111 -> X, Y, Z, W activos

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(120)
        self._poll_timer.timeout.connect(self._send_poll)

        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(70)
        self._jog_timer.timeout.connect(self._jog_heartbeat)
        self._active_jog_cmd = None

        self._build_ui()
        self._apply_style()
        self._refresh_serial_ports()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QHBoxLayout(root)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # ---------------- COLUMNA IZQUIERDA: SERIE + EJES ACTIVOS + DPAD ----------------
        col_left = QVBoxLayout()
        col_left.setSpacing(6)

        # 1. Conexión por Puerto Serie
        gb_comm = QGroupBox("Comunicación Serie UART")
        lay_comm = QGridLayout(gb_comm)
        self.cb_ports = QComboBox()
        self.btn_refresh_ports = QPushButton("↻")
        self.btn_refresh_ports.setFixedWidth(28)
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["115200", "57600", "38400", "9600"])
        self.btn_connect = QPushButton("Conectar")
        self.lbl_status_comm = QLabel("DESCONECTADO")
        self.lbl_status_comm.setStyleSheet("color: #ff4d4f; font-weight: bold;")

        r_port = QHBoxLayout()
        r_port.addWidget(self.cb_ports, 1)
        r_port.addWidget(self.btn_refresh_ports)

        lay_comm.addWidget(QLabel("Puerto COM:"), 0, 0)
        lay_comm.addLayout(r_port, 0, 1)
        lay_comm.addWidget(QLabel("Baudrate:"), 1, 0)
        lay_comm.addWidget(self.cb_baud, 1, 1)
        lay_comm.addWidget(self.btn_connect, 2, 0)
        lay_comm.addWidget(self.lbl_status_comm, 2, 1)
        col_left.addWidget(gb_comm)

        # 2. Selección de Ejes Habilitados en NVS
        gb_axes = QGroupBox("Ejes Presentes en Hardware ($30)")
        lay_ax = QHBoxLayout(gb_axes)
        self.chk_x = QCheckBox("Eje X")
        self.chk_y = QCheckBox("Eje Y")
        self.chk_z = QCheckBox("Eje Z")
        self.chk_w = QCheckBox("Eje W")
        self.btn_save_axes = QPushButton("Guardar Ejes en NVS")
        self.btn_save_axes.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")

        for chk in (self.chk_x, self.chk_y, self.chk_z, self.chk_w):
            chk.setChecked(True)
            lay_ax.addWidget(chk)
        lay_ax.addWidget(self.btn_save_axes)
        col_left.addWidget(gb_axes)

        # 3. Mandos de Jog y Parámetros de Avance Libre
        gb_jog = QGroupBox("Control Manual D-Pad (Modo Libre)")
        v_jog = QVBoxLayout(gb_jog)
        
        r_cfg = QHBoxLayout()
        self.sp_jog_dist = QDoubleSpinBox()
        self.sp_jog_dist.setRange(0.1, 500.0)
        self.sp_jog_dist.setValue(10.0)
        self.sp_jog_feed = QSpinBox()
        self.sp_jog_feed.setRange(10, 5000)
        self.sp_jog_feed.setValue(600)
        self.btn_unlock = QPushButton("Desbloquear ($X)")
        self.btn_unlock.setStyleSheet("background-color: #8957e5; color: white;")

        r_cfg.addWidget(QLabel("Paso (mm):"))
        r_cfg.addWidget(self.sp_jog_dist)
        r_cfg.addWidget(QLabel("Feed:"))
        r_cfg.addWidget(self.sp_jog_feed)
        r_cfg.addWidget(self.btn_unlock)
        v_jog.addLayout(r_cfg)

        dpad_row = QHBoxLayout()
        labels_xy = {"up": "Y+", "down": "Y-", "left": "X-", "right": "X+"}
        self.dpad_xy = DPadPainter(labels=labels_xy)
        self.dpad_zw = VerticalZWControl()
        dpad_row.addWidget(self.dpad_xy, 3)
        dpad_row.addWidget(self.dpad_zw, 2)
        v_jog.addLayout(dpad_row)
        col_left.addWidget(gb_jog, 1)

        main_layout.addLayout(col_left, stretch=48)

        # ---------------- COLUMNA DERECHA: TABLA NVS Y SENSORES ----------------
        col_right = QVBoxLayout()
        col_right.setSpacing(6)

        gb_nvs = QGroupBox("Parámetros del Controlador en Memoria NVS ($$)")
        v_nvs = QVBoxLayout(gb_nvs)

        r_nvs_top = QHBoxLayout()
        self.btn_read_nvs = QPushButton("Leer Parámetros ($$)")
        self.btn_read_nvs.setStyleSheet("background-color: #1f6feb; color: white; font-weight: bold;")
        self.btn_write_selected_nvs = QPushButton("Escribir Parámetro Seleccionado")
        self.btn_write_selected_nvs.setStyleSheet("background-color: #d29922; color: black; font-weight: bold;")
        r_nvs_top.addWidget(self.btn_read_nvs)
        r_nvs_top.addWidget(self.btn_write_selected_nvs)
        r_nvs_top.addStretch(1)
        v_nvs.addLayout(r_nvs_top)

        self.table_nvs = QTableWidget(0, 3)
        self.table_nvs.setHorizontalHeaderLabels(["Parámetro", "Valor Actual", "Descripción"])
        self.table_nvs.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_nvs.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_nvs.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table_nvs.setStyleSheet("background-color: #0d1117; color: #e6edf3; font-family: Consolas;")
        v_nvs.addWidget(self.table_nvs, 1)

        # Sensores de Seguridad
        gb_spec = QGroupBox("Sensores de Seguridad")
        lay_spec = QGridLayout(gb_spec)
        self.chk_probe = QCheckBox("Habilitar Probe G38 ($29=1)")
        self.chk_probe.setChecked(True)
        self.chk_dual = QCheckBox("Límites Duales ($21=1)")
        self.chk_soft = QCheckBox("Soft Limits ($20=1)")
        self.btn_save_spec = QPushButton("Guardar Sensores en NVS")
        self.btn_save_spec.setStyleSheet("background-color: #238636; color: white;")

        lay_spec.addWidget(self.chk_probe, 0, 0)
        lay_spec.addWidget(self.chk_dual, 0, 1)
        lay_spec.addWidget(self.chk_soft, 0, 2)
        lay_spec.addWidget(self.btn_save_spec, 0, 3)
        v_nvs.addWidget(gb_spec)

        col_right.addWidget(gb_nvs)
        main_layout.addLayout(col_right, stretch=52)

        # ---------------- CONEXIONES DE EVENTOS ----------------
        self.btn_refresh_ports.clicked.connect(self._refresh_serial_ports)
        self.btn_connect.clicked.connect(self._toggle_connection)
        self.btn_unlock.clicked.connect(lambda: self._send_cmd("$X"))
        self.btn_read_nvs.clicked.connect(self._request_nvs_dump)
        self.btn_write_selected_nvs.clicked.connect(self._write_table_param)
        self.btn_save_axes.clicked.connect(self._confirm_and_save_axes)
        self.btn_save_spec.clicked.connect(self._confirm_and_save_sensors)

        self.dpad_xy.directionPressed.connect(lambda d: self._start_jog("xy", d))
        self.dpad_xy.directionReleased.connect(self._stop_jog)
        self.dpad_xy.stopPressed.connect(self._on_stop)

        self.dpad_zw.directionPressed.connect(lambda d: self._start_jog("zw", d))
        self.dpad_zw.directionReleased.connect(self._stop_jog)

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #161b22; color: #dbe2ea; font-family: "Segoe UI"; font-size: 11px; }
            QGroupBox { border: 1px solid #2d333b; border-radius: 6px; margin-top: 6px; font-weight: bold; color: #006c6c; padding-top: 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background-color: #0d1117; border: 1px solid #30363d; border-radius: 4px; padding: 3px; color: #e6edf3; }
            QPushButton { background-color: #21262d; border: 1px solid #30363d; border-radius: 4px; padding: 4px 8px; font-weight: bold; color: white; }
            QPushButton:hover { background-color: #30363d; }
            QTableWidget { border: 1px solid #30363d; gridline-color: #21262d; }
        """)

    def _refresh_serial_ports(self):
        self.cb_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.cb_ports.addItem(f"{p.device} ({p.description})", p.device)
        if not ports:
            self.cb_ports.addItem("Sin puertos disponibles", None)

    def _toggle_connection(self):
        if self.worker and self.worker.running:
            self._poll_timer.stop()
            self._jog_timer.stop()
            self.worker.stop()
            self.worker = None
            self.btn_connect.setText("Conectar")
            self.lbl_status_comm.setText("DESCONECTADO")
            self.lbl_status_comm.setStyleSheet("color: #ff4d4f; font-weight: bold;")
        else:
            port = self.cb_ports.currentData()
            if not port:
                QMessageBox.warning(self, "Serie", "Seleccione un puerto COM válido.")
                return
            baud = int(self.cb_baud.currentText())
            self.worker = SerialWorker(port, baud)
            self.worker.connected.connect(self._on_connected)
            self.worker.disconnected.connect(self._on_disconnected)
            self.worker.line_received.connect(self._on_line_received)
            self.worker.start()

    def _on_connected(self):
        self.btn_connect.setText("Desconectar")
        self.lbl_status_comm.setText("CONECTADO (UART)")
        self.lbl_status_comm.setStyleSheet("color: #3fb950; font-weight: bold;")
        self._poll_timer.start()
        QTimer.singleShot(500, self._request_nvs_dump)

    def _on_disconnected(self, msg):
        self._poll_timer.stop()
        self._jog_timer.stop()
        self.btn_connect.setText("Conectar")
        self.lbl_status_comm.setText("DESCONECTADO")
        self.lbl_status_comm.setStyleSheet("color: #ff4d4f; font-weight: bold;")
        QMessageBox.warning(self, "Comunicación Serie", f"Conexión perdida: {msg}")

    def _send_poll(self):
        if self.worker:
            self.worker.send_realtime(b"?")

    def _send_cmd(self, cmd: str):
        if self.worker:
            self.worker.send_line(cmd)

    def _on_stop(self):
        if self.worker:
            self.worker.send_realtime(b"!")
            self.worker.send_realtime(b"\x18")

    def _start_jog(self, group: str, d: str):
        step = self.sp_jog_dist.value()
        feed = self.sp_jog_feed.value()
        ax, dist = "", 0.0

        if group == "xy":
            if d == "up": ax, dist = "Y", step
            elif d == "down": ax, dist = "Y", -step
            elif d == "right": ax, dist = "X", step
            elif d == "left": ax, dist = "X", -step
        else:
            if d == "z_up": ax, dist = "Z", step
            elif d == "z_down": ax, dist = "Z", -step
            elif d == "w_up": ax, dist = "W", step
            elif d == "w_down": ax, dist = "W", -step

        self._active_jog_cmd = f"$J=G91 G21 {ax}{dist:.3f} F{feed}"
        self._send_cmd(self._active_jog_cmd)
        self._jog_timer.start()

    def _jog_heartbeat(self):
        if self._active_jog_cmd and self.worker:
            self._send_cmd(self._active_jog_cmd)

    def _stop_jog(self):
        self._jog_timer.stop()
        self._active_jog_cmd = None
        if self.worker:
            self.worker.send_realtime(bytes([0x85]))

    def _request_nvs_dump(self):
        self.table_nvs.setRowCount(0)
        self._send_cmd("$$")

    def _on_line_received(self, line: str):
        if line.startswith("$") and "=" in line:
            parts = line.split("=")
            p_code = parts[0].strip()
            p_val = parts[1].strip()

            if p_code == "$20":
                self.chk_soft.setChecked(p_val == "1")
            elif p_code == "$21":
                self.chk_dual.setChecked(p_val == "1")
            elif p_code == "$29":
                self.chk_probe.setChecked(p_val == "1")
            elif p_code == "$30":
                try:
                    mask = int(p_val)
                    self.active_axis_mask = mask
                    self.chk_x.setChecked(bool(mask & 1))
                    self.chk_y.setChecked(bool(mask & 2))
                    self.chk_z.setChecked(bool(mask & 4))
                    self.chk_w.setChecked(bool(mask & 8))
                except Exception:
                    pass

            self._update_table_row(p_code, p_val)

    def _update_table_row(self, code: str, val: str):
        descriptions = {
            "$3": "Máscara de inversión de sentido DIR (bits X,Y,Z,W)",
            "$20": "Soft limits habilitados (1=ON, 0=OFF)",
            "$21": "Límites duales por hardware (1=ON, 0=OFF)",
            "$24": "Velocidad de aproximación de Homing (mm/min)",
            "$25": "Velocidad de búsqueda rápida de Homing (mm/min)",
            "$27": "Distancia de separación de switches (Pulloff mm)",
            "$29": "Sensor Touch Probe habilitado (1=ON, 0=OFF)",
            "$30": "Máscara de ejes físicos presentes (bits X,Y,Z,W)",
            "$100": "Resolución pasos/mm Eje X",
            "$101": "Resolución pasos/mm Eje Y",
            "$102": "Resolución pasos/mm Eje Z",
            "$103": "Resolución pasos/mm Eje W",
            "$110": "Velocidad máxima mm/min Eje X",
            "$111": "Velocidad máxima mm/min Eje Y",
            "$112": "Velocidad máxima mm/min Eje Z",
            "$113": "Velocidad máxima mm/min Eje W",
            "$120": "Aceleración mm/s² Eje X",
            "$121": "Aceleración mm/s² Eje Y",
            "$122": "Aceleración mm/s² Eje Z",
            "$123": "Aceleración mm/s² Eje W",
            "$130": "Carrera útil máxima mm Eje X",
            "$131": "Carrera útil máxima mm Eje Y",
            "$132": "Carrera útil máxima mm Eje Z",
            "$133": "Carrera útil máxima mm Eje W",
            "$40": "Compensación de Backlash Eje X (mm)",
            "$41": "Compensación de Backlash Eje Y (mm)",
            "$42": "Compensación de Backlash Eje Z (mm)",
            "$43": "Compensación de Backlash Eje W (mm)",
        }

        for row in range(self.table_nvs.rowCount()):
            if self.table_nvs.item(row, 0).text() == code:
                self.table_nvs.item(row, 1).setText(val)
                return

        row = self.table_nvs.rowCount()
        self.table_nvs.insertRow(row)
        item_code = QTableWidgetItem(code)
        item_code.setFlags(item_code.flags() ^ Qt.ItemIsEditable)
        item_val = QTableWidgetItem(val)
        desc = descriptions.get(code, "Ajuste de firmware")
        item_desc = QTableWidgetItem(desc)
        item_desc.setFlags(item_desc.flags() ^ Qt.ItemIsEditable)

        self.table_nvs.setItem(row, 0, item_code)
        self.table_nvs.setItem(row, 1, item_val)
        self.table_nvs.setItem(row, 2, item_desc)

    def _write_table_param(self):
        row = self.table_nvs.currentRow()
        if row < 0:
            QMessageBox.warning(self, "NVS", "Seleccione una fila en la tabla para modificar.")
            return

        code = self.table_nvs.item(row, 0).text()
        val = self.table_nvs.item(row, 1).text().strip()

        # Doble diálogo de confirmación de seguridad
        r1 = QMessageBox.question(self, "Confirmar Cambio NVS", f"¿Desea grabar {code}={val} en memoria flash permanente?")
        if r1 != QMessageBox.Yes:
            return

        r2 = QMessageBox.warning(
            self, "Alerta de Grabación",
            f"Se escribirá en la memoria NVS del ESP32:\n\n{code} = {val}\n\n¿Confirmar operación de forma definitiva?",
            QMessageBox.Yes | QMessageBox.No
        )
        if r2 == QMessageBox.Yes:
            self._send_cmd(f"{code}={val}")
            QMessageBox.information(self, "NVS", f"Parámetro grabado: {code}={val}")

    def _confirm_and_save_axes(self):
        mask = 0
        if self.chk_x.isChecked(): mask |= 1
        if self.chk_y.isChecked(): mask |= 2
        if self.chk_z.isChecked(): mask |= 4
        if self.chk_w.isChecked(): mask |= 8

        if mask == 0:
            QMessageBox.critical(self, "Error", "Debe haber al menos un eje seleccionado.")
            return

        r1 = QMessageBox.question(self, "Ejes Físicos", f"Se grabará la máscara de hardware: $30={mask}\n¿Continuar?")
        if r1 != QMessageBox.Yes:
            return

        r2 = QMessageBox.warning(
            self, "Alerta de Configuración",
            f"Se modificará la configuración de hardware presente en el ESP32 ($30={mask}).\n¿Confirmar la grabación permanente?",
            QMessageBox.Yes | QMessageBox.No
        )
        if r2 == QMessageBox.Yes:
            self._send_cmd(f"$30={mask}")
            self.active_axis_mask = mask
            QMessageBox.information(self, "NVS", f"Máscara de ejes guardada: $30={mask}")

    def _confirm_and_save_sensors(self):
        p = 1 if self.chk_probe.isChecked() else 0
        d = 1 if self.chk_dual.isChecked() else 0
        s = 1 if self.chk_soft.isChecked() else 0

        r1 = QMessageBox.question(self, "Sensores de Seguridad", f"Ajustes a guardar:\n• Probe ($29): {p}\n• Dual Limits ($21): {d}\n• Soft Limits ($20): {s}\n¿Continuar?")
        if r1 != QMessageBox.Yes:
            return

        r2 = QMessageBox.warning(self, "Seguridad NVS", "¿Confirmar la modificación de sensores en el firmware?", QMessageBox.Yes | QMessageBox.No)
        if r2 == QMessageBox.Yes:
            self._send_cmd(f"$29={p}")
            self._send_cmd(f"$21={d}")
            self._send_cmd(f"$20={s}")
            QMessageBox.information(self, "NVS", "Ajustes de sensores grabados en flash.")

    def closeEvent(self, e):
        self._poll_timer.stop()
        self._jog_timer.stop()
        if self.worker:
            self.worker.stop()
        e.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MaintenanceMainWindow()
    w.show()
    sys.exit(app.exec())