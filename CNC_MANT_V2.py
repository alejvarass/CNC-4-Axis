import sys
import json
import serial
import serial.tools.list_ports

from PySide6.QtCore import Qt, QThread, Signal, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QGroupBox, QTableWidget,
    QTableWidgetItem, QMessageBox, QFileDialog, QHeaderView, QSpinBox
)

NVS_HELP_DESCRIPTIONS = {
    0: ("Eje", "Identificador físico del eje cartesiano o auxiliar (X, Y, Z, W)."),
    1: ("SPM (Steps per mm)", "Resolución cinemática: cantidad de pulsos para avanzar 1.0 mm real."),
    2: ("Max Travel (mm)", "Carrera útil máxima permitida por software (Soft Limit)."),
    3: ("Backoff (stp)", "Pasos que retrocede el carro tras tocar el sensor en búsqueda rápida."),
    4: ("SoftOff (stp)", "Pasos de separación para fijar el origen instrumental (0.000 mm)."),
    5: ("Curva S (1/0)", "Modo de aceleración: '1' con rampa senoidal; '0' velocidad constante."),
    6: ("Seek (us)", "Periodo entre pulsos para búsqueda rápida de Home."),
    7: ("Feed (us)", "Periodo entre pulsos para aproximación fina de Home."),
    8: ("BO us", "Periodo entre pulsos en retroceso de desenganche."),
    9: ("Man us", "Periodo entre pulsos para desplazamiento manual continuo."),
    10: ("Jog us", "Periodo entre pulsos para movimientos incrementales fijos."),
    11: ("Dir Forward", "Nivel lógico en pin DIR para sentido positivo ('1'=HIGH, '0'=LOW).")
}


class SerialWorker(QThread):
    lineReceived = Signal(str)
    connected = Signal()
    disconnected = Signal(str)

    def __init__(self):
        super().__init__()
        self.port_name = ""
        self.baudrate = 115200
        self.ser = None
        self.running = False

    def configure(self, port: str, baud: int = 115200):
        self.port_name = port
        self.baudrate = baud

    def run(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            self.running = True
            self.connected.emit()
            while self.running:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='replace').strip()
                        if line:
                            self.lineReceived.emit(line)
                    except Exception:
                        pass
                self.msleep(10)
        except Exception as e:
            self.disconnected.emit(str(e))
        finally:
            self._close_port()

    def send_line(self, line: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((line.strip() + "\n").encode('utf-8'))
            except Exception:
                pass

    def stop(self):
        self.running = False
        self.wait(200)
        self._close_port()

    def _close_port(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


class MaintenanceWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CNC ESP32 - Mantenimiento NVS & Conexión Red")
        self.setFixedSize(1040, 730)

        self.worker = None
        self.nvs_cache = {}

        self._build_ui()
        self._apply_style()
        self._refresh_ports()

    def _apply_style(self):
        accent = "#006c6c"
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ 
                background-color: #161b22; 
                color: #dbe2ea; 
                font-family: "Segoe UI"; 
                font-size: 11px; 
            }}
            QGroupBox {{ 
                border: 1px solid #2d333b; 
                border-radius: 6px; 
                margin-top: 6px; 
                font-weight: bold; 
                color: {accent}; 
                padding-top: 8px; 
            }}
            QGroupBox::title {{ 
                subcontrol-origin: margin; 
                left: 8px; 
                padding: 0 4px; 
            }}
            QLineEdit, QComboBox, QTableWidget {{ 
                background-color: #0d1117; 
                border: 1px solid #30363d; 
                border-radius: 4px; 
                padding: 3px; 
                color: #e6edf3; 
            }}
            QPushButton {{ 
                background-color: {accent}; 
                border: 1px solid {accent}; 
                border-radius: 4px; 
                padding: 4px 8px; 
                font-weight: bold; 
                color: white; 
            }}
            QPushButton:hover {{ 
                background-color: #008080; 
                border-color: #008080; 
            }}
            QPushButton:disabled {{ 
                background-color: #21262d !important; 
                border: 1px solid #30363d !important; 
                color: #484f58 !important; 
            }}
            QHeaderView::section {{ 
                background-color: #0d1117; 
                color: #58a6ff; 
                font-weight: bold; 
                border: 1px solid #30363d; 
                padding: 4px; 
            }}
            QToolTip {{ 
                background-color: #fff8c5; 
                color: #24292f; 
                border: 1px solid #d4a72c; 
                border-radius: 4px; 
                padding: 6px 8px; 
                font-size: 11px; 
                font-weight: normal; 
            }}
        """)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        lay = QVBoxLayout(root)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        # 1. Conexión Serial USB
        gb_conn = QGroupBox("Conexión Serial USB & IP Estática")
        h_conn = QHBoxLayout(gb_conn)
        self.cb_ports = QComboBox()
        self.cb_ports.setToolTip("Seleccione el puerto serie COM asignado al ESP32 por USB.")
        self.btn_refresh = QPushButton("Refrescar")
        self.btn_refresh.setToolTip("Vuelve a escanear los puertos COM disponibles en la PC.")
        self.btn_connect = QPushButton("Conectar Serial")
        self.btn_disconnect = QPushButton("Desconectar")
        self.btn_disconnect.setEnabled(False)
        self.lbl_status = QLabel("DESCONECTADO")
        self.lbl_status.setStyleSheet("color: #ff7b72; font-weight: bold;")

        h_conn.addWidget(QLabel("COM:"))
        h_conn.addWidget(self.cb_ports, 1)
        h_conn.addWidget(self.btn_refresh)
        h_conn.addWidget(self.btn_connect)
        h_conn.addWidget(self.btn_disconnect)
        h_conn.addWidget(self.lbl_status)
        lay.addWidget(gb_conn)

        # 2. Credenciales Wi-Fi e IP Estática Protegida
        gb_wifi = QGroupBox("Credenciales Wi-Fi & IP Estática Segura")
        v_wifi = QVBoxLayout(gb_wifi)

        row_net = QHBoxLayout()
        self.cb_ssid = QComboBox()
        self.cb_ssid.setEditable(True)
        self.cb_ssid.setToolTip("SSID de la red Wi-Fi almacenado en NVS ('w_ssid').")
        self.btn_scan = QPushButton("Escanear Redes")
        self.btn_scan.setToolTip("Ordena al ESP32 realizar un barrido RF de redes cercanas.")

        self.ed_pass = QLineEdit()
        self.ed_pass.setEchoMode(QLineEdit.Password)
        self.ed_pass.setToolTip("Contraseña WPA2/WPA3 de la red Wi-Fi.")

        self.ed_devname = QLineEdit("CNC-XYZW-NETLOG")
        self.ed_devname.setToolTip("Hostname / Nombre mDNS del dispositivo en la red.")
        
        # IP Fija protegida: bloquea los primeros 3 octetos y permite cambiar el 4to
        ip_box = QHBoxLayout()
        self.lbl_ip_prefix = QLabel("192.168.1.")
        self.lbl_ip_prefix.setStyleSheet("font-weight: bold; color: #58a6ff;")
        self.sp_ip4 = QSpinBox()
        self.sp_ip4.setRange(1, 254)
        self.sp_ip4.setValue(167)
        self.sp_ip4.setFixedWidth(60)
        self.sp_ip4.setToolTip("Modifique únicamente el último octeto (Host ID) de la IP estática.")
        ip_box.addWidget(self.lbl_ip_prefix)
        ip_box.addWidget(self.sp_ip4)

        row_net.addWidget(QLabel("SSID:"))
        row_net.addWidget(self.cb_ssid, 2)
        row_net.addWidget(self.btn_scan)
        row_net.addWidget(QLabel("Pass:"))
        row_net.addWidget(self.ed_pass, 2)
        row_net.addWidget(QLabel("Hostname:"))
        row_net.addWidget(self.ed_devname, 2)
        row_net.addWidget(QLabel("IP Fija:"))
        row_net.addLayout(ip_box)
        v_wifi.addLayout(row_net)

        row_save = QHBoxLayout()
        self.lbl_current = QLabel("Actual: ---")
        self.lbl_current.setStyleSheet("color: #7ee787; font-weight: bold;")
        self.lbl_current.setToolTip("Parámetros de red almacenados actualmente en la NVS.")
        self.btn_save_net = QPushButton("Guardar Red en NVS")
        self.btn_save_net.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")
        self.btn_save_net.setToolTip("Escribe permanentemente las credenciales de red y la IP estática en la flash.")

        row_save.addWidget(self.lbl_current, 1)
        row_save.addWidget(self.btn_save_net)
        v_wifi.addLayout(row_save)
        lay.addWidget(gb_wifi)

        # 3. Parámetros de Ejes en NVS
        gb_nvs = QGroupBox("Parámetros de Ejes en NVS (Pase el cursor sobre los títulos para ayuda)")
        v_nvs = QVBoxLayout(gb_nvs)

        self.table_nvs = QTableWidget()
        self.table_nvs.setColumnCount(12)
        headers = [
            "Eje", "SPM", "Max Travel", "Backoff", "SoftOff",
            "Curva S", "Seek", "Feed", "BO us", "Man us", "Jog us", "Dir"
        ]
        self.table_nvs.setHorizontalHeaderLabels(headers)
        self.table_nvs.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        for col_idx, (title, desc) in NVS_HELP_DESCRIPTIONS.items():
            header_item = self.table_nvs.horizontalHeaderItem(col_idx)
            if header_item:
                header_item.setToolTip(f"<b>{title}</b><br>{desc}")

        v_nvs.addWidget(self.table_nvs, 1)

        row_btns = QHBoxLayout()
        self.btn_read = QPushButton("Leer NVS")
        self.btn_read.setToolTip("Carga y vuelca todos los parámetros almacenados en la NVS del ESP32.")
        self.btn_write = QPushButton("Guardar Fila Seleccionada")
        self.btn_write.setStyleSheet("background-color: #8957e5; color: white;")
        self.btn_write.setToolTip("Sobrescribe en la NVS los valores modificados de la fila del eje seleccionado.")
        self.btn_restart = QPushButton("Reiniciar ESP32")
        self.btn_restart.setStyleSheet("background-color: #a42424; color: white;")
        self.btn_restart.setToolTip("Envía orden de reinicio por hardware al microcontrolador.")

        row_btns.addWidget(self.btn_read)
        row_btns.addWidget(self.btn_write)
        row_btns.addStretch(1)
        row_btns.addWidget(self.btn_restart)
        v_nvs.addLayout(row_btns)

        lay.addWidget(gb_nvs, 1)

        # Conexiones de eventos
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_connect.clicked.connect(self._connect)
        self.btn_disconnect.clicked.connect(lambda: self.worker.stop() if self.worker else None)
        self.btn_scan.clicked.connect(lambda: self.worker.send_line("SCAN_WIFI") if self.worker else None)
        self.btn_save_net.clicked.connect(self._confirm_and_save_net)
        self.btn_read.clicked.connect(lambda: self.worker.send_line("DUMP_NVS") if self.worker else None)
        self.btn_write.clicked.connect(self._confirm_and_save_axis)
        self.btn_restart.clicked.connect(lambda: self.worker.send_line("RESTART_ESP") if self.worker else None)

    def _refresh_ports(self):
        self.cb_ports.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_ports.addItem(f"{p.device} ({p.description})", p.device)

    def _connect(self):
        port = self.cb_ports.currentData()
        if not port:
            QMessageBox.warning(self, "Puerto", "Seleccione un puerto COM válido.")
            return

        self.worker = SerialWorker()
        self.worker.configure(port, 115200)
        self.worker.connected.connect(lambda: (
            self.btn_connect.setEnabled(False),
            self.btn_disconnect.setEnabled(True),
            self.lbl_status.setText("CONECTADO"),
            self.lbl_status.setStyleSheet("color: #3fb950; font-weight: bold;"),
            QTimer.singleShot(400, lambda: self.worker.send_line("DUMP_NVS"))
        ))
        self.worker.disconnected.connect(lambda: (
            self.btn_connect.setEnabled(True),
            self.btn_disconnect.setEnabled(False),
            self.lbl_status.setText("DESCONECTADO"),
            self.lbl_status.setStyleSheet("color: #ff7b72; font-weight: bold;")
        ))
        self.worker.lineReceived.connect(self._on_line)
        self.worker.start()

    def _confirm_and_save_net(self):
        ssid = self.cb_ssid.currentText().strip()
        pwd = self.ed_pass.text().strip()
        devname = self.ed_devname.text().strip()
        ip4 = self.sp_ip4.value()

        if not ssid:
            QMessageBox.warning(self, "Validación", "El campo SSID no puede estar vacío.")
            return

        msg = f"¿Está seguro de guardar la red en NVS?\n\n• SSID: {ssid}\n• Hostname: {devname}\n• IP Fija: 192.168.1.{ip4}"
        if QMessageBox.warning(self, "Confirmar Red", msg, QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
            self.worker.send_line(f"SET_WIFI_NVS|{ssid}|{pwd}|{devname}|{ip4}")

    def _confirm_and_save_axis(self):
        row = self.table_nvs.currentRow()
        if row < 0:
            QMessageBox.warning(self, "NVS", "Seleccione una fila en la tabla de ejes.")
            return

        ax = self.table_nvs.item(row, 0).text().lower()
        vals = [self.table_nvs.item(row, i).text() for i in range(1, 12)]
        if QMessageBox.warning(self, "Confirmar Eje", f"¿Sobrescribir parámetros del eje {ax.upper()} en NVS?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
            self.worker.send_line(f"LOAD_AXIS_PARAM|{ax}|" + "|".join(vals))

    def _on_line(self, line: str):
        if line.startswith("WIFI_LIST|"):
            self.cb_ssid.clear()
            for s in line.split("|")[1].split(","):
                if s.strip():
                    self.cb_ssid.addItem(s.strip())
        elif line.startswith("ACK|SET_WIFI_NVS|OK"):
            if QMessageBox.question(self, "Reiniciar", "Red guardada con éxito en NVS.\n\n¿Desea reiniciar el ESP32 ahora?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.worker.send_line("RESTART_ESP")
        elif line.startswith("ACK|LOAD_AXIS_PARAM|OK"):
            QMessageBox.information(self, "NVS", "Parámetros de eje actualizados correctamente.")
        elif line.startswith("NVS_DATA|"):
            try:
                data = json.loads(line.split("|", 1)[1])
                self.cb_ssid.setCurrentText(data.get("ssid", ""))
                self.ed_pass.setText(data.get("pass", ""))
                self.ed_devname.setText(data.get("devname", "CNC-XYZW"))
                ip4 = data.get("ip_oct4", 167)
                self.sp_ip4.setValue(ip4)
                self.lbl_current.setText(f"NVS IP: {data.get('current_ip')} | SSID: {data.get('ssid')} | IP Fija: 192.168.1.{ip4}")
                
                axes = data.get("axes", {})
                self.table_nvs.setRowCount(len(axes))
                for r, (k, v) in enumerate(axes.items()):
                    row_v = [
                        k.upper(), v.get("spm"), v.get("max"), v.get("bo"), v.get("so"),
                        v.get("sc"), v.get("seek"), v.get("feed"), v.get("bo_us"),
                        v.get("man_us"), v.get("jog_us"), v.get("dfl")
                    ]
                    for c, val in enumerate(row_v):
                        item = QTableWidgetItem(str(val))
                        title, desc = NVS_HELP_DESCRIPTIONS.get(c, ("", ""))
                        item.setToolTip(f"<b>{title} (Eje {k.upper()}):</b><br>{desc}")
                        if c == 0:
                            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                            item.setTextAlignment(Qt.AlignCenter)
                            item.setForeground(QColor("#ffb347"))
                        self.table_nvs.setItem(r, c, item)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Fallo al procesar NVS: {e}")

    def closeEvent(self, event):
        if self.worker:
            self.worker.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MaintenanceWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()