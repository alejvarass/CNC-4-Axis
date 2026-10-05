"""
Controlador CNC XYZW - Versión 6.5.1 (Pure GRBL Engine)
Programa Principal de Control - 4 Tabs Operativas (Sin Mantenimiento)
"""

import sys
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QSpinBox, QDoubleSpinBox, QGroupBox,
    QTabWidget, QCheckBox, QSlider, QComboBox, QFormLayout,
    QSizePolicy, QTextEdit
)

from grbl_driver import GrblTcpWorker
from cnc_widgets import (
    CameraMeasurementWidget, DPadPainter, VerticalZWControl, AxisBlock
)
from cnc_tabs import AxisTab, SequenceProgrammerWidget, GCodeRunnerWidget

CLIENT_VERSION = "6.5.1"


class MainWindow(QMainWindow):
    AXES = ["x", "y", "z", "w"]

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"Controlador CNC XYZW - Versión {CLIENT_VERSION} (Pure GRBL Engine)")
        self.setMinimumSize(1300, 780)
        self.resize(1400, 840)

        self.worker = None
        self.mpos = [0.0, 0.0, 0.0, 0.0]
        self.wpos = [0.0, 0.0, 0.0, 0.0]
        self.spm_cache = {"x": 568.0, "y": 568.0, "z": 568.0, "w": 568.0}
        self.dir_invert_mask = 0

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(100)
        self._poll_timer.timeout.connect(self._send_status_query)

        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(70)
        self._jog_timer.timeout.connect(self._jog_heartbeat_tick)
        self._active_jog_cmd = None

        self._spindle_debounce_timer = QTimer(self)
        self._spindle_debounce_timer.setSingleShot(True)
        self._spindle_debounce_timer.setInterval(150)
        self._spindle_debounce_timer.timeout.connect(self._send_debounced_spindle)

        self._user_disconnect = False
        self._reconnect_attempts = 0
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setSingleShot(True)
        self._reconnect_timer.timeout.connect(self._try_reconnect)

        self._build_ui()
        self._apply_style()
        self._update_controls_interlock()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        left_container = QWidget()
        left_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        center_container = QWidget()
        center_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_container = QWidget()
        right_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout.addWidget(left_container, stretch=28)
        layout.addWidget(center_container, stretch=36)
        layout.addWidget(right_container, stretch=36)

        # ---------------- COLUMNA 1: CONEXIÓN & CALIBRACIÓN NVS ----------------
        l_lay = QVBoxLayout(left_container)
        l_lay.setContentsMargins(0, 0, 0, 0)
        l_lay.setSpacing(4)

        gb_conn = QGroupBox("Configuración de Red & Protocolo")
        form_conn = QFormLayout(gb_conn)
        form_conn.setContentsMargins(6, 6, 6, 6)
        form_conn.setSpacing(4)

        self.ed_ip = QLineEdit("192.168.1.167")
        self.ed_port = QSpinBox()
        self.ed_port.setRange(1, 65535)
        self.ed_port.setValue(5000)

        self.btn_connect = QPushButton("Conectar")
        self.btn_disconnect = QPushButton("Desconectar")
        self.btn_disconnect.setEnabled(False)

        row_c_btns = QHBoxLayout()
        row_c_btns.setSpacing(4)
        row_c_btns.addWidget(self.btn_connect)
        row_c_btns.addWidget(self.btn_disconnect)

        self.chk_autounlock = QCheckBox("Desbloqueo auto ($X) al conectar")
        self.chk_autounlock.setChecked(False)
        self.chk_autoreconnect = QCheckBox("Auto-reconectar si cae TCP")
        self.chk_autoreconnect.setChecked(True)

        form_conn.addRow("IP Host:", self.ed_ip)
        form_conn.addRow("Puerto:", self.ed_port)
        form_conn.addRow(row_c_btns)
        form_conn.addRow("", self.chk_autounlock)
        form_conn.addRow("", self.chk_autoreconnect)
        l_lay.addWidget(gb_conn)

        gb_tabs = QGroupBox("Parámetros & Calibración de Ejes")
        vtabs = QVBoxLayout(gb_tabs)
        vtabs.setContentsMargins(4, 4, 4, 4)
        self.tabs_nvs = QTabWidget()
        self.axis_tabs = {}
        for a in self.AXES:
            tab = AxisTab(a, self)
            self.axis_tabs[a] = tab
            self.tabs_nvs.addTab(tab, a.upper())
        vtabs.addWidget(self.tabs_nvs)
        l_lay.addWidget(gb_tabs, 1)

        gb_diag = QGroupBox("Espacio Auxiliar / Diagnóstico")
        v_diag = QVBoxLayout(gb_diag)
        self.lbl_diag_info = QLabel("Motor GRBL v1.1h puro activo.\nComandos en tiempo real ?, !, ~, 0x18, 0x85.")
        self.lbl_diag_info.setStyleSheet("color: #8b949e; font-style: italic;")
        self.lbl_diag_info.setWordWrap(True)
        v_diag.addWidget(self.lbl_diag_info)
        l_lay.addWidget(gb_diag, 1)

        # ---------------- COLUMNA 2: ESTADO GENERAL + BLOQUES DRO ----------------
        c_lay = QVBoxLayout(center_container)
        c_lay.setContentsMargins(0, 0, 0, 0)
        c_lay.setSpacing(4)

        gb_sys = QGroupBox("Estado del Sistema")
        sys_grid = QGridLayout(gb_sys)
        sys_grid.setContentsMargins(4, 4, 4, 4)
        sys_grid.setHorizontalSpacing(4)
        sys_grid.setVerticalSpacing(2)

        self.lbl_conn_text = QLabel("DESCONECTADO")
        self.lbl_conn_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff4d4f;")
        self.lbl_actuators_text = QLabel("DESHABILITADOS")
        self.lbl_actuators_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff4d4f;")
        self.btn_enable_actuators = QPushButton("Activar Actuadores ($X)")

        self.lbl_machine_state_v = QLabel("---")
        self.lbl_machine_state_v.setStyleSheet("font-size: 13px; font-weight: bold;")
        self.btn_all_homing = QPushButton("ALL HOMING ($H)")
        self.btn_all_homing.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")

        sys_grid.addWidget(QLabel("Conexión:"), 0, 0)
        sys_grid.addWidget(self.lbl_conn_text, 0, 1)
        sys_grid.addWidget(QLabel("Drivers:"), 1, 0)
        sys_grid.addWidget(self.lbl_actuators_text, 1, 1)
        sys_grid.addWidget(self.btn_enable_actuators, 1, 2)
        sys_grid.addWidget(QLabel("Máquina:"), 2, 0)
        sys_grid.addWidget(self.lbl_machine_state_v, 2, 1)
        sys_grid.addWidget(self.btn_all_homing, 2, 2)
        c_lay.addWidget(gb_sys)

        self.axis_blocks = {}
        for a in self.AXES:
            blk = AxisBlock(a, self)
            self.axis_blocks[a] = blk
            c_lay.addWidget(blk)
        c_lay.addStretch(1)

        # ---------------- COLUMNA 3: 4 PESTAÑAS EXACTAS + DPAD ----------------
        r_lay = QVBoxLayout(right_container)
        r_lay.setContentsMargins(0, 0, 0, 0)
        r_lay.setSpacing(4)

        self.right_tabs = QTabWidget()

        # Tab 1: Cámara USB
        tab_cam_container = QWidget()
        cam_layout = QVBoxLayout(tab_cam_container)
        cam_layout.setContentsMargins(3, 3, 3, 3)
        cam_layout.setSpacing(2)

        self.cam_widget = CameraMeasurementWidget()
        cam_layout.addWidget(self.cam_widget, 1)

        cam_toolbar = QWidget()
        cam_toolbar.setStyleSheet("background-color: #0d1117; border: 1px solid #30363d; border-radius: 4px; padding: 2px;")
        cam_tool_layout = QVBoxLayout(cam_toolbar)
        cam_tool_layout.setContentsMargins(3, 2, 3, 2)
        cam_tool_layout.setSpacing(2)

        r_cam1 = QHBoxLayout()
        r_cam1.setSpacing(3)
        self.cb_cam_idx = QComboBox()
        self.cb_cam_idx.addItems(["CAM 0", "CAM 1", "CAM 2"])
        self.cb_cam_idx.setFixedWidth(68)
        self.btn_cam_toggle = QPushButton("Iniciar Cámara")
        self.btn_cam_toggle.setFixedWidth(95)

        self.sl_cam_zoom = QSlider(Qt.Horizontal)
        self.sl_cam_zoom.setRange(100, 500)
        self.sl_cam_zoom.setValue(100)
        self.sl_cam_zoom.setFixedWidth(75)
        self.lbl_cam_zoom = QLabel("1.0x")
        self.lbl_cam_zoom.setFixedWidth(28)

        self.sp_cam_px = QDoubleSpinBox()
        self.sp_cam_px.setRange(1.0, 5000.0)
        self.sp_cam_px.setValue(100.0)
        self.sp_cam_px.setFixedWidth(58)
        self.btn_set_scale = QPushButton("Fijar Escala")
        self.btn_set_scale.setFixedWidth(70)

        r_cam1.addWidget(self.cb_cam_idx)
        r_cam1.addWidget(self.btn_cam_toggle)
        r_cam1.addWidget(QLabel("Zoom:"))
        r_cam1.addWidget(self.sl_cam_zoom)
        r_cam1.addWidget(self.lbl_cam_zoom)
        r_cam1.addWidget(QLabel("Px/10mm:"))
        r_cam1.addWidget(self.sp_cam_px)
        r_cam1.addWidget(self.btn_set_scale)

        r_cam2 = QHBoxLayout()
        r_cam2.setSpacing(3)
        self.chk_grid = QCheckBox("Grid")
        self.chk_grid.setChecked(True)
        self.sl_grid_size = QSlider(Qt.Horizontal)
        self.sl_grid_size.setRange(10, 150)
        self.sl_grid_size.setValue(40)
        self.sl_grid_size.setFixedWidth(80)
        self.lbl_grid_val = QLabel("40px")
        self.lbl_grid_val.setFixedWidth(32)

        self.btn_clear_meas = QPushButton("Borrar Medición")
        self.btn_clear_meas.setStyleSheet("background-color: #3b202a; border-color: #6e273b; color: #ff7b72;")

        r_cam2.addWidget(self.chk_grid)
        r_cam2.addWidget(QLabel("Paso:"))
        r_cam2.addWidget(self.sl_grid_size)
        r_cam2.addWidget(self.lbl_grid_val)
        r_cam2.addStretch(1)
        r_cam2.addWidget(self.btn_clear_meas)

        cam_tool_layout.addLayout(r_cam1)
        cam_tool_layout.addLayout(r_cam2)
        cam_layout.addWidget(cam_toolbar)
        self.right_tabs.addTab(tab_cam_container, "Visor Óptico USB")

        # Tab 2: Secuenciador
        self.seq_widget = SequenceProgrammerWidget(self)
        self.right_tabs.addTab(self.seq_widget, "Secuenciador CNC")

        # Tab 3: G-Code Runner
        self.gcode_widget = GCodeRunnerWidget(self)
        self.right_tabs.addTab(self.gcode_widget, "G-code")

        # Tab 4: Terminal MDI
        tab_term = QWidget()
        v_t = QVBoxLayout(tab_term)
        self.txt_term = QTextEdit()
        self.txt_term.setReadOnly(True)
        self.txt_term.setStyleSheet("background-color: #0d1117; color: #7ee787; font-family: Consolas; font-size: 11px;")
        v_t.addWidget(self.txt_term, 1)

        r_mdi = QHBoxLayout()
        self.ed_mdi = QLineEdit()
        self.ed_mdi.setPlaceholderText("Comando GRBL o G-code (ej: G0 X10, $$, $H)...")
        self.btn_send_mdi = QPushButton("Enviar")
        r_mdi.addWidget(self.ed_mdi, 1)
        r_mdi.addWidget(self.btn_send_mdi)
        v_t.addLayout(r_mdi)
        self.right_tabs.addTab(tab_term, "Terminal MDI ($$)")

        r_lay.addWidget(self.right_tabs, 54)

        # Panel D-Pad Inferior
        gb_dpad = QGroupBox("Control Manual D-Pad XY / Control ZW")
        v_dp = QVBoxLayout(gb_dpad)
        v_dp.setContentsMargins(4, 2, 4, 2)

        r_sp = QHBoxLayout()
        self.btn_m3 = QPushButton("Husillo ON (M3)")
        self.btn_m3.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")
        self.btn_m5 = QPushButton("Husillo OFF (M5)")
        self.btn_m5.setStyleSheet("background-color: #a42424; color: white; font-weight: bold;")

        self.sl_spindle = QSlider(Qt.Horizontal)
        self.sl_spindle.setRange(0, 1000)
        self.sl_spindle.setValue(1000)
        self.lbl_spindle_val = QLabel("S: 1000")
        r_sp.addWidget(self.btn_m3)
        r_sp.addWidget(self.btn_m5)
        r_sp.addWidget(self.sl_spindle)
        r_sp.addWidget(self.lbl_spindle_val)
        v_dp.addLayout(r_sp)

        r_jp = QHBoxLayout()
        self.sp_jog_dist = QDoubleSpinBox()
        self.sp_jog_dist.setRange(0.01, 100.0)
        self.sp_jog_dist.setValue(5.0)

        self.sp_jog_feed = QSpinBox()
        self.sp_jog_feed.setRange(10, 5000)
        self.sp_jog_feed.setValue(600)

        self.btn_direct_stop = QPushButton("PARADA DE EMERGENCIA (! + 0x18)")
        self.btn_direct_stop.setStyleSheet("background-color: #b02a2a; color: white; font-weight: bold;")

        r_jp.addWidget(QLabel("Paso Jog (mm):"))
        r_jp.addWidget(self.sp_jog_dist)
        r_jp.addWidget(QLabel("Feed (mm/min):"))
        r_jp.addWidget(self.sp_jog_feed)
        r_jp.addWidget(self.btn_direct_stop)
        v_dp.addLayout(r_jp)

        dpad_row = QHBoxLayout()
        labels_xy = {"up": "Y+", "down": "Y-", "left": "X-", "right": "X+"}
        self.dpad_xy = DPadPainter(labels=labels_xy)
        self.dpad_zw = VerticalZWControl()

        dpad_row.addWidget(self.dpad_xy, 3)
        dpad_row.addWidget(self.dpad_zw, 2)
        v_dp.addLayout(dpad_row, 1)

        r_lay.addWidget(gb_dpad, 46)

        # ---------------- CONEXIONES DE EVENTOS ----------------
        self.btn_connect.clicked.connect(self._connect)
        self.btn_disconnect.clicked.connect(self._disconnect)
        self.btn_all_homing.clicked.connect(lambda: self.send_gcode("$H"))
        self.btn_enable_actuators.clicked.connect(lambda: self.send_gcode("$X"))

        self.btn_m3.clicked.connect(lambda: self.send_gcode(f"M3 S{self.sl_spindle.value()}"))
        self.btn_m5.clicked.connect(lambda: self.send_gcode("M5"))
        self.sl_spindle.valueChanged.connect(self._on_spindle_slider_changed)

        self.btn_cam_toggle.clicked.connect(self._toggle_camera)
        self.chk_grid.toggled.connect(self.cam_widget.set_grid_enabled)
        self.sl_grid_size.valueChanged.connect(lambda v: (self.lbl_grid_val.setText(f"{v}px"), self.cam_widget.set_grid_size(v)))
        self.sl_cam_zoom.valueChanged.connect(self._on_zoom_changed)
        self.btn_set_scale.clicked.connect(lambda: self.cam_widget.set_px_per_mm(self.sp_cam_px.value()))
        self.btn_clear_meas.clicked.connect(self.cam_widget.clear_measurements)

        self.dpad_xy.directionPressed.connect(lambda d: self._start_jog("xy", d))
        self.dpad_xy.directionReleased.connect(self._stop_jog)
        self.dpad_xy.stopPressed.connect(self.on_stop_clicked)
        self.btn_direct_stop.clicked.connect(self.on_stop_clicked)

        self.dpad_zw.directionPressed.connect(lambda d: self._start_jog("zw", d))
        self.dpad_zw.directionReleased.connect(self._stop_jog)

        self.btn_send_mdi.clicked.connect(self._send_mdi)
        self.ed_mdi.returnPressed.connect(self._send_mdi)

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
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ 
                background-color: #0d1117; 
                border: 1px solid #30363d; 
                border-radius: 4px; 
                padding: 3px; 
                color: #e6edf3; 
            }}
            QTabWidget::pane {{ 
                border: 1px solid #30363d; 
                background-color: #0d1117; 
                border-radius: 4px; 
            }}
            QTabBar::tab {{ 
                background: #0d1117; 
                border: 1px solid #30363d; 
                padding: 4px 8px; 
                margin-right: 2px; 
                font-size: 11px; 
                color: #8b949e; 
            }}
            QTabBar::tab:selected {{ 
                background: #12303b; 
                color: #7ee787; 
                font-weight: bold; 
                border-bottom: 2px solid #58a6ff; 
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

    def _update_controls_interlock(self):
        is_conn = bool(self.worker and self.worker.running)
        is_alarm = "alarm" in self.lbl_machine_state_v.text().lower()

        self.btn_connect.setEnabled(not is_conn)
        self.btn_disconnect.setEnabled(is_conn)
        self.btn_all_homing.setEnabled(is_conn and not is_alarm)
        self.btn_enable_actuators.setEnabled(is_conn)

        self.dpad_xy.setEnabled(is_conn and not is_alarm)
        self.dpad_zw.setEnabled(is_conn and not is_alarm)
        self.seq_widget.btn_run.setEnabled(is_conn and not is_alarm and self.seq_widget.list_orders.count() > 0)
        self.gcode_widget.btn_run.setEnabled(is_conn and not is_alarm and len(self.gcode_widget.lines) > 0)

        for blk in self.axis_blocks.values():
            blk.setEnabled(is_conn and not is_alarm)

    def _connect(self):
        self._user_disconnect = False
        self._reconnect_attempts = 0
        self.worker = GrblTcpWorker()
        self.worker.configure(self.ed_ip.text().strip(), int(self.ed_port.value()))
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)
        self.worker.status_received.connect(self._on_status)
        self.worker.ack_received.connect(self._on_ack)
        self.worker.message_received.connect(self._on_message)
        self.worker.start()

    def _disconnect(self):
        self._user_disconnect = True
        self._reconnect_timer.stop()
        if self.worker:
            self.worker.stop()
        self._on_disconnected("Desconectado por el usuario.")

    def _on_connected(self):
        self._reconnect_attempts = 0
        self.lbl_conn_text.setText("CONECTADO")
        self.lbl_conn_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #3fb950;")
        self._poll_timer.start()

        if self.chk_autounlock.isChecked():
            self.send_gcode("$X")
        self.send_gcode("$$")
        self._update_controls_interlock()

    def _on_disconnected(self, msg):
        self._poll_timer.stop()
        self._jog_timer.stop()
        self.lbl_conn_text.setText("DESCONECTADO")
        self.lbl_conn_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff4d4f;")
        self.lbl_actuators_text.setText("DESHABILITADOS")
        self.lbl_actuators_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff4d4f;")
        self.lbl_machine_state_v.setText("OFFLINE")
        self.lbl_machine_state_v.setStyleSheet("font-size: 13px; font-weight: bold; color: #8b949e;")
        self.gcode_widget._stop()
        self._update_controls_interlock()

        if not self._user_disconnect and self.chk_autoreconnect.isChecked():
            if self._reconnect_attempts < 5:
                self._reconnect_attempts += 1
                self.lbl_conn_text.setText(f"RECONECTANDO ({self._reconnect_attempts}/5)")
                self.lbl_conn_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #f2cc60;")
                self._reconnect_timer.start(1000 * self._reconnect_attempts)

    def _try_reconnect(self):
        if not self._user_disconnect:
            self._connect()

    def _send_status_query(self):
        if self.worker:
            self.worker.send_realtime(b"?")

    def send_gcode(self, line: str):
        if self.worker:
            self.txt_term.append(f"> {line}")
            self.worker.send_line(line)

    def send_realtime(self, ch):
        if self.worker:
            ok = self.worker.send_realtime(ch)
            if not ok:
                self.txt_term.append("[WARN] Fallo de envío en tiempo real.")

    def _send_mdi(self):
        cmd = self.ed_mdi.text().strip()
        if cmd:
            self.send_gcode(cmd)
            self.ed_mdi.clear()

    def _on_spindle_slider_changed(self, v):
        self.lbl_spindle_val.setText(f"S: {v}")
        self._spindle_debounce_timer.start()

    def _send_debounced_spindle(self):
        v = self.sl_spindle.value()
        self.send_gcode(f"S{v}")

    def on_stop_clicked(self):
        self.send_realtime(b"!")
        self.send_realtime(b"\x18")
        self._update_controls_interlock()

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
        self.send_gcode(self._active_jog_cmd)
        self._jog_timer.start()

    def _jog_heartbeat_tick(self):
        if self._active_jog_cmd and self.worker:
            self.worker.send_line(self._active_jog_cmd)

    def _stop_jog(self):
        self._jog_timer.stop()
        self._active_jog_cmd = None
        self.send_realtime(bytes([0x85]))

    def _on_status(self, data: dict):
        st = data.get("state", "Idle")
        pn = data.get("pn", "")
        pn_str = f" | Pn:{pn}" if pn else ""

        self.lbl_machine_state_v.setText(f"{st.upper()}{pn_str}")

        if st.lower() == "alarm":
            self.lbl_machine_state_v.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff4d4f;")
            self.lbl_actuators_text.setText("DESHABILITADOS")
            self.lbl_actuators_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff4d4f;")
        elif st.lower() in ("run", "jog"):
            self.lbl_machine_state_v.setStyleSheet("font-size: 13px; font-weight: bold; color: #58a6ff;")
            self.lbl_actuators_text.setText("HABILITADOS")
            self.lbl_actuators_text.setStyleSheet("font-size: 13px; font-weight: bold; color: #3fb950;")
        elif st.lower() == "hold":
            self.lbl_machine_state_v.setStyleSheet("font-size: 13px; font-weight: bold; color: #f2cc60;")
        else:
            self.lbl_machine_state_v.setStyleSheet("font-size: 13px; font-weight: bold; color: #3fb950;")

        self.wpos = data.get("wpos", [0.0]*4)
        self.mpos = data.get("mpos", [0.0]*4)
        is_moving = st.lower() in ("run", "jog")

        for i, a in enumerate(self.AXES):
            self.axis_blocks[a].update_data_direct(
                self.wpos[i], self.mpos[i], is_moving, "moving" if is_moving else "stop",
                st.lower() != "alarm", ""
            )

        self.seq_widget.notify_status(data)
        self._update_controls_interlock()

    def _on_ack(self, ok: bool, msg: str):
        self.txt_term.append(f"< {msg}")
        self.seq_widget.notify_ack(ok)
        self.gcode_widget.notify_ack(ok, msg)

    def _on_message(self, line: str):
        self.txt_term.append(f"[MSG] {line}")
        if line.startswith("$") and "=" in line:
            try:
                k, v = line.split("=")
                p_num = int(k[1:])
                val = float(v)

                # Adaptación dinámica de ejes presentes en hardware ($30)
                if p_num == 30:
                    mask = int(val)
                    for idx, a in enumerate(self.AXES):
                        is_present = bool(mask & (1 << idx))
                        if a in self.axis_blocks:
                            self.axis_blocks[a].setVisible(is_present)
                        if a in self.axis_tabs:
                            self.tabs_nvs.setTabVisible(idx, is_present)

                elif 100 <= p_num <= 103:
                    ax = self.AXES[p_num - 100]
                    self.spm_cache[ax] = val
                    self.axis_tabs[ax].sp_spm.setValue(val)
                elif 110 <= p_num <= 113:
                    ax = self.AXES[p_num - 110]
                    self.axis_tabs[ax].sp_max_rate.setValue(val)
                elif 120 <= p_num <= 123:
                    ax = self.AXES[p_num - 120]
                    self.axis_tabs[ax].sp_accel.setValue(val)
                elif 130 <= p_num <= 133:
                    ax = self.AXES[p_num - 130]
                    self.axis_tabs[ax].sp_max_travel.setValue(val)
                elif 40 <= p_num <= 43:
                    ax = self.AXES[p_num - 40]
                    self.axis_tabs[ax].sp_backlash.setValue(val)
                elif p_num == 3:
                    self.dir_invert_mask = int(val)
                elif p_num == 24:
                    for t in self.axis_tabs.values(): t.sp_hfeed.setValue(val)
                elif p_num == 25:
                    for t in self.axis_tabs.values(): t.sp_hseek.setValue(val)
                elif p_num == 27:
                    for t in self.axis_tabs.values(): t.sp_hpull.setValue(val)
            except Exception:
                pass

    def get_axis_spm(self, axis: str) -> float:
        return self.spm_cache.get(axis.lower(), 568.0)

    def _toggle_camera(self):
        if self.cam_widget.camera_running():
            self.cam_widget.stop_camera()
            self.btn_cam_toggle.setText("Iniciar Cámara")
        else:
            self.cam_widget.start_camera(self.cb_cam_idx.currentIndex())
            self.btn_cam_toggle.setText("Detener Cámara")

    def _on_zoom_changed(self, v):
        factor = float(v) / 100.0
        self.lbl_cam_zoom.setText(f"{factor:.1f}x")
        self.cam_widget.set_zoom(v)

    def closeEvent(self, e):
        self._poll_timer.stop()
        self._jog_timer.stop()
        self._reconnect_timer.stop()
        if self.worker:
            self.worker.stop()
        if self.cam_widget:
            self.cam_widget.stop_camera()
        e.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()