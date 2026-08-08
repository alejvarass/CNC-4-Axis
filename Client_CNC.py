import sys
import json
import socket
from datetime import datetime
import math
from threading import Thread, Lock

from PySide6.QtCore import Qt, QThread, Signal, QSize, QPointF, QRectF, QTimer
from PySide6.QtGui import QColor, QTextCursor, QFont, QPainter, QPen, QPainterPath
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QSpinBox,
    QDoubleSpinBox, QGroupBox, QMessageBox, QSplitter, QFormLayout, QTabWidget
)


def build_message(msg_type: str, msg_id: str, seq: int, payload: dict) -> dict:
    return {"type": msg_type, "id": msg_id, "seq": seq, "payload": payload}


def now_str():
    return datetime.now().strftime("%H:%M:%S")


class DPadPainter(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)
    stopPressed = Signal()

    def __init__(self, labels=None, parent=None):
        super().__init__(parent)
        self.setMinimumSize(230, 230)
        self._pressed = False
        self._active = None
        self.setMouseTracking(True)
        
        self.labels = labels or {
            "up": "↑",
            "down": "↓",
            "left": "←",
            "right": "→"
        }

    def _geom(self):
        w, h = self.width(), self.height()
        c = QPointF(w / 2, h / 2)
        outer = min(w, h) * 0.46
        inner = min(w, h) * 0.23
        stop_r = inner
        return c, outer, inner, stop_r

    def _hit(self, pos):
        c, outer, inner, stop_r = self._geom()
        x = pos.x() - c.x()
        y = pos.y() - c.y()
        r = math.hypot(x, y)

        if r <= stop_r:
            return "stop"
        if r > outer:
            return None

        ang = math.degrees(math.atan2(-y, x))
        if ang < 0:
            ang += 360

        if 45 <= ang < 135:
            return "up"
        if 135 <= ang < 225:
            return "left"
        if 225 <= ang < 315:
            return "down"
        return "right"

    def mousePressEvent(self, e):
        if not self.isEnabled() or e.button() != Qt.LeftButton:
            return
        zone = self._hit(e.position())
        if zone is None:
            return

        self._pressed = True
        self._active = zone

        if zone == "stop":
            self.stopPressed.emit()
        else:
            self.directionPressed.emit(zone)

        self.update()

    def mouseMoveEvent(self, e):
        if not self.isEnabled() or not self._pressed:
            return
        zone = self._hit(e.position())
        if zone in ("up", "down", "left", "right", "stop") and zone != self._active:
            old_zone = self._active
            self._active = zone
            if old_zone and old_zone != "stop":
                self.directionReleased.emit(old_zone)
            if zone == "stop":
                self.stopPressed.emit()
            else:
                self.directionPressed.emit(zone)
            self.update()

    def mouseReleaseEvent(self, e):
        if self._pressed:
            old = self._active
            self._pressed = False
            self._active = None
            if old and old != "stop":
                self.directionReleased.emit(old)
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        c, outer, inner, stop_r = self._geom()

        p.fillRect(self.rect(), QColor("#161b22"))
        p.setPen(QPen(QColor("#2f4953"), 1))
        p.setBrush(QColor("#0f1a20"))
        p.drawEllipse(c, outer, outer)

        gap = 4.0

        def draw_sector(start_deg, span_deg, key):
            adj_start = start_deg + gap / 2.0
            adj_span = span_deg - gap

            outer_rect = QRectF(c.x() - outer, c.y() - outer, outer * 2, outer * 2)
            inner_rect = QRectF(c.x() - inner, c.y() - inner, inner * 2, inner * 2)

            path = QPainterPath()
            path.arcMoveTo(outer_rect, adj_start)
            path.arcTo(outer_rect, adj_start, adj_span)
            path.arcTo(inner_rect, adj_start + adj_span, -adj_span)
            path.closeSubpath()

            color_idle = QColor("#12232c") if self.isEnabled() else QColor("#1c2128")
            color_act = QColor("#1f4958")
            p.setPen(QPen(QColor("#2f4953"), 1))
            p.setBrush(color_act if self._active == key else color_idle)
            p.drawPath(path)

        draw_sector(45, 90, "up")
        draw_sector(135, 90, "left")
        draw_sector(225, 90, "down")
        draw_sector(315, 90, "right")

        p.setPen(QPen(QColor("#b02a2a"), 2))
        p.setBrush(QColor("#a42424") if self._active == "stop" else QColor("#7d1b1b"))
        p.drawEllipse(c, stop_r, stop_r)

        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", max(10, int(inner * 0.35)), QFont.Bold))
        
        p.drawText(QRectF(c.x() - 25, c.y() - outer + 12, 50, 30), Qt.AlignCenter, self.labels.get("up", "↑"))
        p.drawText(QRectF(c.x() - outer + 5, c.y() - 15, 45, 30), Qt.AlignCenter, self.labels.get("left", "←"))
        p.drawText(QRectF(c.x() + outer - 50, c.y() - 15, 45, 30), Qt.AlignCenter, self.labels.get("right", "→"))
        p.drawText(QRectF(c.x() - 25, c.y() + outer - 38, 50, 30), Qt.AlignCenter, self.labels.get("down", "↓"))

        p.setFont(QFont("Segoe UI", max(11, int(stop_r * 0.40)), QFont.Bold))
        p.drawText(QRectF(c.x() - stop_r, c.y() - stop_r, stop_r * 2, stop_r * 2), Qt.AlignCenter, "STOP")
        p.end()


class VerticalZWControl(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(165, 253)
        self._pressed = False
        self._active = None
        self.setMouseTracking(True)

    def _get_outer_oval_rect(self):
        w, h = self.width(), self.height()
        oval_w = min(w * 0.77, 143)
        oval_h = min(h * 0.95, 242)
        x = (w - oval_w) / 2
        y = (h - oval_h) / 2
        return QRectF(x, y, oval_w, oval_h)

    def _get_inner_oval_rect(self, outer_r, gap):
        h_sec = (outer_r.height() - gap * 3) / 4.0
        top_w = outer_r.top() + h_sec + gap
        height_w = (h_sec * 2) + gap
        
        inset_x = outer_r.width() * 0.08
        return QRectF(
            outer_r.left() + inset_x,
            top_w,
            outer_r.width() - (inset_x * 2),
            height_w
        )

    def _hit(self, pos):
        outer_r = self._get_outer_oval_rect()
        if not outer_r.contains(pos):
            return None

        gap = 6.0
        inner_r = self._get_inner_oval_rect(outer_r, gap)

        if inner_r.contains(pos):
            return "w_up" if pos.y() < inner_r.center().y() else "w_down"

        return "z_up" if pos.y() < outer_r.center().y() else "z_down"

    def mousePressEvent(self, e):
        if not self.isEnabled() or e.button() != Qt.LeftButton:
            return
        zone = self._hit(e.position())
        if zone is None:
            return

        self._pressed = True
        self._active = zone
        self.directionPressed.emit(zone)
        self.update()

    def mouseMoveEvent(self, e):
        if not self.isEnabled() or not self._pressed:
            return
        zone = self._hit(e.position())
        if zone != self._active:
            if self._active:
                self.directionReleased.emit(self._active)
            self._active = zone
            if zone:
                self.directionPressed.emit(zone)
            self.update()

    def mouseReleaseEvent(self, e):
        if self._pressed:
            if self._active:
                self.directionReleased.emit(self._active)
            self._pressed = False
            self._active = None
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        p.fillRect(self.rect(), QColor("#161b22"))

        outer_r = self._get_outer_oval_rect()
        outer_radius = outer_r.width() / 2.0

        gap = 6.0
        inner_r = self._get_inner_oval_rect(outer_r, gap)
        inner_radius = inner_r.width() / 2.0

        bg_outer_path = QPainterPath()
        bg_outer_path.addRoundedRect(outer_r, outer_radius, outer_radius)

        bg_inner_path = QPainterPath()
        bg_inner_path.addRoundedRect(inner_r, inner_radius, inner_radius)

        p.setPen(QPen(QColor("#2f4953"), 1))
        p.setBrush(QColor("#0f1a20"))
        p.drawPath(bg_outer_path)

        color_idle = QColor("#12232c") if self.isEnabled() else QColor("#1c2128")
        color_act = QColor("#1f4958")

        h_sec = (outer_r.height() - gap * 3) / 4.0

        sec_z_up = QRectF(outer_r.left(), outer_r.top(), outer_r.width(), h_sec)
        path_z_up = QPainterPath()
        path_z_up.addRect(sec_z_up)
        clipped_z_up = path_z_up.intersected(bg_outer_path)

        p.setPen(QPen(QColor("#2f4953"), 1))
        p.setBrush(color_act if self._active == "z_up" else color_idle)
        p.drawPath(clipped_z_up)

        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", 12, QFont.Bold))
        p.drawText(sec_z_up, Qt.AlignCenter, "Z+")

        sec_z_down = QRectF(outer_r.left(), outer_r.top() + (h_sec + gap) * 3, outer_r.width(), h_sec)
        path_z_down = QPainterPath()
        path_z_down.addRect(sec_z_down)
        clipped_z_down = path_z_down.intersected(bg_outer_path)

        p.setPen(QPen(QColor("#2f4953"), 1))
        p.setBrush(color_act if self._active == "z_down" else color_idle)
        p.drawPath(clipped_z_down)

        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", 12, QFont.Bold))
        p.drawText(sec_z_down, Qt.AlignCenter, "Z-")

        p.setPen(QPen(QColor("#00f0ff") if self.isEnabled() else QColor("#2f4953"), 1, Qt.DashLine))
        p.setBrush(QColor("#0d181e"))
        p.drawPath(bg_inner_path)

        sec_w_up = QRectF(inner_r.left(), inner_r.top(), inner_r.width(), (inner_r.height() - gap) / 2.0)
        path_w_up = QPainterPath()
        path_w_up.addRect(sec_w_up)
        clipped_w_up = path_w_up.intersected(bg_inner_path)

        p.setPen(QPen(QColor("#2f4953"), 1))
        p.setBrush(color_act if self._active == "w_up" else color_idle)
        p.drawPath(clipped_w_up)

        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", 11, QFont.Bold))
        p.drawText(sec_w_up, Qt.AlignCenter, "W+")

        sec_w_down = QRectF(inner_r.left(), inner_r.top() + ((inner_r.height() - gap) / 2.0) + gap, inner_r.width(), (inner_r.height() - gap) / 2.0)
        path_w_down = QPainterPath()
        path_w_down.addRect(sec_w_down)
        clipped_w_down = path_w_down.intersected(bg_inner_path)

        p.setPen(QPen(QColor("#2f4953"), 1))
        p.setBrush(color_act if self._active == "w_down" else color_idle)
        p.drawPath(clipped_w_down)

        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", 11, QFont.Bold))
        p.drawText(sec_w_down, Qt.AlignCenter, "W-")

        p.end()


class TcpWorker(QThread):
    connected = Signal()
    disconnected = Signal(str)
    parsed_msg = Signal(dict)
    tx_line = Signal(str)
    error = Signal(str)

    def __init__(self):
        super().__init__()
        self.host = "192.168.1.167"
        self.port = 5000
        self.sock = None
        self.running = False
        self.read_timeout = 0.05
        self._rx_buffer = b""

    def configure(self, host: str, port: int):
        self.host = host
        self.port = port

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.read_timeout)
            self.sock.connect((self.host, self.port))
            self.running = True
            self.connected.emit()

            while self.running:
                try:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        self.running = False
                        self.disconnected.emit("Servidor cerró conexión")
                        break

                    self._rx_buffer += chunk
                    while b"\n" in self._rx_buffer:
                        line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            obj = json.loads(line.decode("utf-8", errors="replace"))
                            self.parsed_msg.emit(obj)
                        except Exception:
                            pass

                except socket.timeout:
                    continue
                except Exception as e:
                    self.running = False
                    self.disconnected.emit(f"RX error: {e}")
                    break

        except Exception as e:
            self.error.emit(f"No se pudo conectar: {e}")
        finally:
            self._close_socket()

    def send_obj(self, obj: dict):
        if not self.sock:
            raise RuntimeError("Socket no conectado")
        line = json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n"
        try:
            self.sock.sendall(line.encode("utf-8"))
            self.tx_line.emit(line.strip())
        except (BrokenPipeError, ConnectionResetError) as e:
            self.running = False
            self.disconnected.emit(f"Error enviando: {e}")

    def stop(self):
        self.running = False
        self._close_socket()

    def _close_socket(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class AxisTab(QWidget):
    def __init__(self, axis_name: str, main_window):
        super().__init__()
        self.axis = axis_name.lower()
        self.main_window = main_window
        self.current_spm = 568.0
        self.raw_step_count = 0

        self.calib_step = 0
        self.dist1 = 0.0
        self.steps1 = 0
        self.spm1 = 0.0
        
        self.dist_total = 0.0
        self.steps_total = 0
        self.spm2 = 0.0
        
        self.initial_manual_us = 100

        lay = QVBoxLayout(self)
        gb = QGroupBox(f"Calibración Promedio de 2 Pasadas - Eje {self.axis.upper()}")
        form = QFormLayout(gb)

        self.lbl_pasos_acumulados = QLabel("0 pasos")
        self.lbl_pasos_acumulados.setStyleSheet("font-weight: bold; color: #58a6ff; font-size: 14px;")

        self.lbl_calculated_spm = QLabel("---")
        self.lbl_calculated_spm.setStyleSheet("font-weight: bold; color: #00f0ff; font-size: 14px;")

        self.btn_calculate_preview = QPushButton("Calcular (Valor steps/mm)")
        self.btn_calculate_preview.setEnabled(False)

        self.sp_distancia = QDoubleSpinBox()
        self.sp_distancia.setRange(0.0, 5000.0)
        self.sp_distancia.setDecimals(3)
        self.sp_distancia.setEnabled(False)

        self.lbl_calib_instructions = QLabel("Listo para iniciar calibración.")
        self.lbl_calib_instructions.setStyleSheet("color: #e6edf3; font-weight: bold;")
        self.lbl_calib_instructions.setWordWrap(True)

        self.btn_start_wizard = QPushButton("Iniciar Calibración (Pasada 1)")
        self.btn_confirm_pass = QPushButton("Confirmar Pasada")
        self.btn_confirm_pass.setEnabled(False)

        row_steps_calc = QHBoxLayout()
        row_steps_calc.addWidget(self.lbl_pasos_acumulados, 1)
        row_steps_calc.addWidget(self.btn_calculate_preview, 1)

        form.addRow("Etapa de Calibración:", self.lbl_calib_instructions)
        form.addRow("Pasos leídos ESP32:", row_steps_calc)
        form.addRow("Valor Calculado (steps/mm):", self.lbl_calculated_spm)
        form.addRow("Distancia medida (mm):", self.sp_distancia)
        form.addRow(self.btn_start_wizard)
        form.addRow(self.btn_confirm_pass)

        div = QWidget()
        div.setFixedHeight(1)
        div.setStyleSheet("background-color: #30363d;")
        form.addRow(div)

        self.sp_manual_us = QSpinBox()
        self.sp_manual_us.setRange(5, 20000)
        self.sp_manual_us.setValue(100)
        self.btn_set_profile = QPushButton("Guardar Velocidad")

        form.addRow("Velocidad manual (us):", self.sp_manual_us)
        form.addRow(self.btn_set_profile)

        lay.addWidget(gb, 1)

        self.btn_start_wizard.clicked.connect(self._start_calibration_wizard)
        self.btn_confirm_pass.clicked.connect(self._confirm_pass_measurement)
        self.btn_calculate_preview.clicked.connect(self._calculate_preview)

    def set_steps_count(self, steps: int):
        self.raw_step_count = steps
        self.lbl_pasos_acumulados.setText(f"{steps} pasos")

    def _calculate_preview(self):
        steps = float(abs(self.raw_step_count))
        mm = float(self.sp_distancia.value())
        if steps > 0 and mm > 0.0:
            spm = steps / mm
            self.lbl_calculated_spm.setText(f"{spm:.4f} steps/mm")
        else:
            QMessageBox.warning(self, "Cálculo no disponible", "Se requieren pasos acumulados registrados por la ESP32 (>0) y una distancia medida física mayor a 0 mm.")
            self.lbl_calculated_spm.setText("---")

    def _start_calibration_wizard(self):
        ax = self.axis
        act_ok = bool(self.main_window.last_status.get("actuators_enabled", False))
        if not act_ok:
            QMessageBox.warning(self, "Actuadores Deshabilitados", "Debe energizar los actuadores antes de iniciar la calibración.")
            return

        d = self.main_window.last_status.get("axes", {}).get(ax, {}) if isinstance(self.main_window.last_status.get("axes"), dict) else {}
        if not d.get("homed", False):
            QMessageBox.warning(self, "Homing Requerido", f"Debe ejecutar el Homing del eje {ax.upper()} antes de calibrar.")
            return

        self.calib_step = 1
        self.initial_manual_us = int(self.sp_manual_us.value())
        
        self.main_window.send_axis_cmd(ax, "reset_step_counter", {})
        
        self.sp_distancia.setEnabled(True)
        self.sp_distancia.setValue(0.0)
        self.btn_confirm_pass.setEnabled(True)
        self.btn_confirm_pass.setText("Confirmar Pasada 1")
        self.btn_calculate_preview.setEnabled(True)
        self.btn_start_wizard.setEnabled(False)
        
        self.lbl_calib_instructions.setText(
            "PASADA 1:\nDesplace el eje con el D-Pad en (+).\nMida la distancia D1 (mm), ingrésela abajo y presione 'Confirmar Pasada 1'."
        )

    def _confirm_pass_measurement(self):
        val = float(self.sp_distancia.value())
        if val <= 0.0:
            QMessageBox.warning(self, "Valor Inválido", "La distancia registrada debe ser mayor a 0 mm.")
            return

        if self.calib_step == 1:
            self.dist1 = val
            self.steps1 = abs(self.raw_step_count)
            
            if self.steps1 <= 0:
                QMessageBox.warning(self, "Pasos Nulos", "No se registraron pasos de movimiento para la Pasada 1.")
                self.calib_step = 0
                self.btn_start_wizard.setEnabled(True)
                self.btn_confirm_pass.setEnabled(False)
                self.lbl_calib_instructions.setText("Calibración abortada. Listo para reintentar.")
                return
                
            self.spm1 = float(self.steps1) / self.dist1
            self.calib_step = 2

            self.sp_distancia.setValue(0.0)
            self.btn_confirm_pass.setText("Confirmar Pasada 2")
            
            self.lbl_calib_instructions.setText(
                f"PASADA 1 COMPLETADA: D1={self.dist1:.3f}mm, Pasos={self.steps1}, SPM1={self.spm1:.4f}\n\n"
                "PASADA 2:\nDesplace NUEVAMENTE el eje en (+) con el D-Pad.\nMida la distancia acumulada total D_total (mm), ingrésela abajo y presione 'Confirmar Pasada 2'."
            )

        elif self.calib_step == 2:
            self.dist_total = val
            self.steps_total = abs(self.raw_step_count)

            if self.steps_total <= 0 or self.dist_total <= 0.0:
                QMessageBox.warning(self, "Error de Pasos", "No se registraron pasos acumulados o distancia válida en la Pasada 2.")
                self.calib_step = 0
                self.btn_start_wizard.setEnabled(True)
                self.btn_confirm_pass.setEnabled(False)
                self.lbl_calib_instructions.setText("Calibración abortada. Listo para reintentar.")
                return

            self.spm2 = float(self.steps_total) / self.dist_total
            self.spm_final = (self.spm1 + self.spm2) / 2.0

            fecha_calib = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.main_window.send_axis_cmd(self.axis, "set_calibration_axis", {
                "steps_per_mm": float(self.spm_final),
                "max_travel": float(self.dist_total),
                "last_calibration": fecha_calib
            })

            self.current_spm = self.spm_final
            self.lbl_calculated_spm.setText(f"{self.spm_final:.4f} steps/mm")
            self.main_window.sp_main_max_travel.setValue(self.dist_total)

            QMessageBox.information(
                self, "Calibración Completada y Guardada",
                f"Calibración de 2 pasadas finalizada para eje [{self.axis.upper()}]:\n\n"
                f"--- Pasada 1 ---\n"
                f"Distancia D1: {self.dist1:.3f} mm | Pasos 1: {self.steps1}\n"
                f"SPM1: {self.spm1:.4f} steps/mm\n\n"
                f"--- Pasada 2 (Total) ---\n"
                f"Distancia Total: {self.dist_total:.3f} mm | Pasos Totales: {self.steps_total}\n"
                f"SPM2: {self.spm2:.4f} steps/mm\n\n"
                f"PROMEDIO FINAL: {self.spm_final:.4f} steps/mm"
            )

            self.calib_step = 0
            self.btn_start_wizard.setEnabled(True)
            self.btn_confirm_pass.setEnabled(False)
            self.btn_confirm_pass.setText("Confirmar Pasada")
            self.btn_calculate_preview.setEnabled(False)
            self.sp_distancia.setEnabled(False)
            self.lbl_calib_instructions.setText("Calibración completada. Sistema listo para usar.")


class MainWindow(QMainWindow):
    AXES = ["x", "y", "z", "w"]
    
    update_status_signal = Signal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 XYZW Controller - I+D CNC Professional")

        self.worker = None
        self.seq = 1
        self.last_status = {}
        self.status_lock = Lock()

        self._dpad_active = False
        self._dpad_axis_group = "xy"
        self._dpad_dir = None
        self._auto_scroll_terminal = True
        self.setting_direction_in_progress = False
        self._is_moving = False

        self._build_ui()
        self._apply_style()
        self._set_initial_disconnected_ui()
        
        # Timer para actualizar status rápidamente
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.poll_status)
        self.status_timer.start(50)
        
        # Timer para actualizar posición localmente de forma suave
        self.pos_timer = QTimer()
        self.pos_timer.timeout.connect(self._update_position_smoothly)
        self.pos_timer.start(20)
        
        # Signal para actualización thread-safe
        self.update_status_signal.connect(self._on_status_update)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()
        super().keyPressEvent(event)

    def _set_initial_disconnected_ui(self):
        self.lbl_conn_text.setText("DESCONECTADO")
        self._set_label_color_state(self.lbl_conn_text, "red")
        
        self.lbl_homed_text.setText("---")
        self._set_label_color_state(self.lbl_homed_text, "default")
        
        self.lbl_actuators_text.setText("---")
        self._set_label_color_state(self.lbl_actuators_text, "default")
        
        self.lbl_pos.setText("---")
        self.lbl_woff.setText("---")
        self.lbl_target_pos.setText("---")
        self.lbl_machine_state_v.setText("---")
        self.lbl_moving_text.setText("---")
        self.lbl_dir_text.setText("---")
        self.lbl_last_error.setText("---")
        self.lbl_calib_date.setText("---")
        
        self._update_controls_interlock()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        main_layout.addWidget(splitter, 1)

        # LEFT PANEL
        left = QWidget()
        left_layout = QVBoxLayout(left)

        gb_conn = QGroupBox("Conexión TCP")
        form_conn = QFormLayout(gb_conn)

        self.ed_host = QLineEdit("192.168.1.167")
        self.ed_port = QSpinBox()
        self.ed_port.setRange(1, 65535)
        self.ed_port.setValue(5000)

        self.btn_connect = QPushButton("Conectar")
        self.btn_disconnect = QPushButton("Desconectar")
        self.btn_disconnect.setEnabled(False)

        row_conn = QHBoxLayout()
        row_conn.addWidget(self.btn_connect)
        row_conn.addWidget(self.btn_disconnect)

        form_conn.addRow("Host/IP:", self.ed_host)
        form_conn.addRow("Puerto:", self.ed_port)
        form_conn.addRow(row_conn)
        left_layout.addWidget(gb_conn)

        gb_tabs = QGroupBox("Ejes")
        vtabs = QVBoxLayout(gb_tabs)
        self.tabs = QTabWidget()
        self.axis_tabs = {}
        for a in self.AXES:
            t = AxisTab(a, self)
            self.axis_tabs[a] = t
            self.tabs.addTab(t, a.upper())
        vtabs.addWidget(self.tabs)
        left_layout.addWidget(gb_tabs, 1)

        # CENTER PANEL
        center = QWidget()
        center_layout = QVBoxLayout(center)
        top_center = QHBoxLayout()

        gb_state = QGroupBox("Estado del Controlador")
        state_grid = QGridLayout(gb_state)

        font_status_top = QFont("Segoe UI", 11, QFont.Bold)

        self.lbl_conn_text = QLabel("DESCONECTADO")
        self.lbl_conn_text.setFont(font_status_top)

        self.lbl_homed_text = QLabel("---")
        self.lbl_homed_text.setFont(font_status_top)

        self.lbl_actuators_text = QLabel("---")
        self.lbl_actuators_text.setFont(font_status_top)
        
        self.lbl_pos = QLabel("---")
        self.lbl_pos.setStyleSheet("font-size: 18px; font-weight: bold; color: #00f0ff;")
        
        self.lbl_woff = QLabel("---")
        self.lbl_woff.setStyleSheet("font-size: 16px; font-weight: bold; color: #00f0ff;")

        self.lbl_target_pos = QLabel("---")
        self.lbl_target_pos.setStyleSheet("font-size: 16px; font-weight: bold; color: #dbe2ea;")

        self.lbl_machine_state_v = QLabel("---")
        self.lbl_moving_text = QLabel("---")

        self.lbl_axis_active = QLabel("X")
        self.lbl_dir_text = QLabel("---")
        self.lbl_dir_text.setStyleSheet("font-size: 14px; font-weight: bold; color: #ff9500;")
        self.lbl_last_error = QLabel("---")
        self.lbl_last_error.setWordWrap(True)

        self.lbl_calib_date = QLabel("---")

        lbl_title_conn = QLabel("Conexión:"); lbl_title_conn.setFont(font_status_top)
        lbl_title_homing = QLabel("Estado Homing:"); lbl_title_homing.setFont(font_status_top)
        lbl_title_actuators = QLabel("Actuadores:"); lbl_title_actuators.setFont(font_status_top)

        state_grid.addWidget(lbl_title_conn, 0, 0); state_grid.addWidget(self.lbl_conn_text, 0, 1)
        state_grid.addWidget(lbl_title_homing, 1, 0); state_grid.addWidget(self.lbl_homed_text, 1, 1)
        state_grid.addWidget(lbl_title_actuators, 2, 0); state_grid.addWidget(self.lbl_actuators_text, 2, 1)
        state_grid.addWidget(QLabel("Posición Actual:"), 3, 0); state_grid.addWidget(self.lbl_pos, 3, 1)
        state_grid.addWidget(QLabel("Work Offset (woff):"), 4, 0); state_grid.addWidget(self.lbl_woff, 4, 1)
        state_grid.addWidget(QLabel("Posición Objetivo:"), 5, 0); state_grid.addWidget(self.lbl_target_pos, 5, 1)
        state_grid.addWidget(QLabel("Estado Máquina:"), 6, 0); state_grid.addWidget(self.lbl_machine_state_v, 6, 1)
        state_grid.addWidget(QLabel("Eje Activo:"), 7, 0); state_grid.addWidget(self.lbl_axis_active, 7, 1)
        state_grid.addWidget(QLabel("Estado Movimiento:"), 8, 0); state_grid.addWidget(self.lbl_moving_text, 8, 1)
        state_grid.addWidget(QLabel("Sentido:"), 9, 0); state_grid.addWidget(self.lbl_dir_text, 9, 1)
        state_grid.addWidget(QLabel("Último Error:"), 10, 0); state_grid.addWidget(self.lbl_last_error, 10, 1)
        state_grid.addWidget(QLabel("Última Calibración:"), 11, 0); state_grid.addWidget(self.lbl_calib_date, 11, 1)

        gb_jog = QGroupBox("Control Operativo de Eje")
        jog_grid = QGridLayout(gb_jog)

        self.sp_main_max_travel = QDoubleSpinBox()
        self.sp_main_max_travel.setRange(1.0, 5000.0)
        self.sp_main_max_travel.setValue(110.0) 

        self.sp_main_backoff_steps = QSpinBox()
        self.sp_main_backoff_steps.setRange(1, 100000)
        self.sp_main_backoff_steps.setValue(1200)

        self.sp_soft_offset_steps = QSpinBox()
        self.sp_soft_offset_steps.setRange(1, 200000)
        self.sp_soft_offset_steps.setValue(4000)

        self.sp_main_hseek = QSpinBox()
        self.sp_main_hseek.setRange(5, 20000)
        self.sp_main_hseek.setValue(250)

        self.sp_main_hbo = QSpinBox()
        self.sp_main_hbo.setRange(5, 20000)
        self.sp_main_hbo.setValue(400)

        self.btn_enable_actuators = QPushButton("Activar actuadores")
        self.btn_enable_actuators.setCheckable(True)
        
        self.btn_stop = QPushButton("STOP / PARADA EMERGENCIA")
        self.btn_stop.setStyleSheet("background-color: #a42424; color: white; font-weight: bold;")

        self.btn_zero = QPushButton("Set Zero")
        self.btn_set_homing_vars = QPushButton("Guardar Config")
        self.btn_home = QPushButton("HOMING")
        self.btn_home.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")

        self.sp_move_abs = QDoubleSpinBox()
        self.sp_move_abs.setRange(-5000.0, 5000.0)
        self.sp_move_abs.setDecimals(3)
        self.btn_move_abs = QPushButton("Mover Absoluto")

        self.sp_jog = QDoubleSpinBox()
        self.sp_jog.setRange(0.001, 500.0)
        self.sp_jog.setDecimals(3)
        self.sp_jog.setValue(10.0)

        self.btn_jog_neg = QPushButton("Atrás (-)")
        self.btn_jog_pos = QPushButton("Adelante (+)")

        # VELOCIDAD MANUAL para D-Pads
        self.sp_dpad_speed = QSpinBox()
        self.sp_dpad_speed.setRange(5, 20000)
        self.sp_dpad_speed.setValue(150)
        self.lbl_dpad_speed = QLabel(f"Velocidad D-Pad: {self.sp_dpad_speed.value()} µs")

        jog_grid.addWidget(self.btn_enable_actuators, 0, 0)
        jog_grid.addWidget(self.btn_stop, 0, 1)
        
        jog_grid.addWidget(QLabel("Max travel (mm)"), 1, 0)
        jog_grid.addWidget(self.sp_main_max_travel, 1, 1)
        jog_grid.addWidget(QLabel("Home backoff (pasos)"), 2, 0)
        jog_grid.addWidget(self.sp_main_backoff_steps, 2, 1)
        jog_grid.addWidget(QLabel("Soft Limit (pasos)"), 3, 0)
        jog_grid.addWidget(self.sp_soft_offset_steps, 3, 1)
        
        jog_grid.addWidget(QLabel("Homing seek (us)"), 4, 0)
        jog_grid.addWidget(self.sp_main_hseek, 4, 1)
        jog_grid.addWidget(QLabel("Homing backoff (us)"), 5, 0)
        jog_grid.addWidget(self.sp_main_hbo, 5, 1)

        jog_grid.addWidget(self.btn_zero, 6, 0)
        jog_grid.addWidget(self.btn_set_homing_vars, 6, 1)
        jog_grid.addWidget(self.btn_home, 7, 0, 1, 2)

        jog_grid.addWidget(QLabel("Posición Absoluta (mm):"), 8, 0)
        jog_grid.addWidget(self.sp_move_abs, 8, 1)
        jog_grid.addWidget(self.btn_move_abs, 9, 0, 1, 2)

        jog_grid.addWidget(QLabel("Delta Relativo (mm):"), 10, 0)
        jog_grid.addWidget(self.sp_jog, 10, 1)
        jog_grid.addWidget(self.btn_jog_neg, 11, 0)
        jog_grid.addWidget(self.btn_jog_pos, 11, 1)

        jog_grid.addWidget(self.lbl_dpad_speed, 12, 0)
        jog_grid.addWidget(self.sp_dpad_speed, 12, 1)

        top_center.addWidget(gb_state, 2)
        center_layout.addLayout(top_center, 1)
        center_layout.addWidget(gb_jog, 1)

        # RIGHT PANEL
        right = QWidget()
        right_layout = QVBoxLayout(right)
        top_right = QHBoxLayout()

        gb_dpad = QGroupBox("D-Pad XY / Control ZW Óvalo")
        dpad_layout = QHBoxLayout(gb_dpad)

        labels_xy = {
            "up": "Y+",
            "down": "Y-",
            "left": "X-",
            "right": "X+"
        }

        self.dpad_xy = DPadPainter(labels=labels_xy)
        self.dpad_zw = VerticalZWControl()

        dpad_layout.addWidget(self.dpad_xy)
        dpad_layout.addWidget(self.dpad_zw)

        top_right.addWidget(gb_dpad, 3)

        gb_log = QGroupBox("Terminal")
        log_layout = QVBoxLayout(gb_log)
        row_log = QHBoxLayout()
        
        self.btn_status = QPushButton("get_status")
        self.btn_clear_log = QPushButton("Limpiar")
        self.btn_pause_log = QPushButton("Pausar")
        self.btn_pause_log.setCheckable(True)

        row_log.addWidget(self.btn_status)
        row_log.addWidget(self.btn_clear_log)
        row_log.addWidget(self.btn_pause_log)
        row_log.addStretch(1)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFont(QFont("Consolas", 9))
        self.txt_log.setMaximumHeight(150)

        log_layout.addLayout(row_log)
        log_layout.addWidget(self.txt_log)

        right_layout.addLayout(top_right, 1)
        right_layout.addWidget(gb_log, 1)

        splitter.addWidget(left)
        splitter.addWidget(center)
        splitter.addWidget(right)
        splitter.setSizes([380, 450, 730])

        # CONEXIONES
        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)
        self.btn_status.clicked.connect(self.poll_status)
        self.btn_home.clicked.connect(self.send_home_axis)
        self.btn_zero.clicked.connect(self.send_zero_axis)
        self.btn_set_homing_vars.clicked.connect(self.send_homing_vars)
        self.btn_enable_actuators.clicked.connect(self.send_enable_actuators)
        
        self.btn_stop.clicked.connect(self.on_stop_clicked)

        self.btn_move_abs.clicked.connect(self.send_move_abs)
        self.btn_jog_neg.clicked.connect(lambda: self.safe_send_relative(-abs(float(self.sp_jog.value()))))
        self.btn_jog_pos.clicked.connect(lambda: self.safe_send_relative(+abs(float(self.sp_jog.value()))))

        self.btn_clear_log.clicked.connect(self.txt_log.clear)
        self.btn_pause_log.toggled.connect(self._toggle_terminal_autoscroll)

        self.sp_dpad_speed.valueChanged.connect(self._on_dpad_speed_changed)

        self.tabs.currentChanged.connect(self._on_tab_changed)

        for ax_name, tab in self.axis_tabs.items():
            tab.btn_set_profile.clicked.connect(lambda _, a=ax_name, t=tab: self.on_set_profile_and_speed(a, t))

        self.dpad_xy.directionPressed.connect(lambda d: self._dpad_press_group("xy", d))
        self.dpad_xy.directionReleased.connect(lambda d: self._dpad_release_direction("xy", d))
        self.dpad_xy.stopPressed.connect(self.on_motion_stop_clicked)

        self.dpad_zw.directionPressed.connect(self._zw_vertical_press)
        self.dpad_zw.directionReleased.connect(self._zw_vertical_release)

        self._on_tab_changed(0)

    def _toggle_terminal_autoscroll(self, paused: bool):
        self._auto_scroll_terminal = not paused
        if paused:
            self.btn_pause_log.setText("Reanudar")
            self.btn_pause_log.setStyleSheet("background-color: #8c6d1f; color: white;")
        else:
            self.btn_pause_log.setText("Pausar")
            self.btn_pause_log.setStyleSheet("")

    def _on_dpad_speed_changed(self, value):
        self.lbl_dpad_speed.setText(f"Velocidad D-Pad: {value} µs")
        for ax_name in self.AXES:
            self.send_axis_cmd(ax_name, "set_manual_speed_axis", {"delay_us": value})

    def _apply_style(self):
        accent = "#006c6c"
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background-color: #161b22; color: #dbe2ea; }}
            QGroupBox {{
                border: 1px solid #2d333b;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: 700;
                color: {accent};
            }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; }}
            QLineEdit, QSpinBox, QDoubleSpinBox, QTextEdit, QTabWidget::pane {{
                background-color: #0d1117;
                border: 1px solid #30363d;
                border-radius: 6px;
                padding: 6px;
                color: #e6edf3;
            }}
            QTabBar::tab {{
                background: #0d1117;
                border: 1px solid #30363d;
                padding: 8px 12px;
                margin-right: 2px;
            }}
            QTabBar::tab:selected {{ background: #12303b; color: #7ee787; }}
            QPushButton {{
                background-color: {accent};
                border: 1px solid {accent};
                border-radius: 6px;
                padding: 7px 10px;
                font-weight: 700;
                color: white;
            }}
            QPushButton:hover {{ background-color: #008080; border-color: #008080; }}
            QPushButton:disabled {{ background-color: #3a3f44; border-color: #3a3f44; color: #9aa4af; }}

            QLabel[stateColor="green"] {{ color: #3fb950; font-weight: 700; }}
            QLabel[stateColor="red"] {{ color: #ff4d4f; font-weight: 700; }}
            QLabel[stateColor="yellow"] {{ color: #f2cc60; font-weight: 700; }}
            QLabel[stateColor="blue"] {{ color: #58a6ff; font-weight: 700; }}
            QLabel[stateColor="default"] {{ color: #dbe2ea; font-weight: 700; }}
        """)

    def _set_label_color_state(self, label: QLabel, state: str):
        label.setProperty("stateColor", state)
        label.style().unpolish(label)
        label.style().polish(label)
        label.update()

    def _update_controls_interlock(self):
        is_tcp_connected = bool(self.worker and self.worker.running)
        
        ax = self._axis_from_tab()
        d = self.last_status.get("axes", {}).get(ax, {}) if isinstance(self.last_status.get("axes"), dict) else {}
        
        actuators_ok = bool(self.last_status.get("actuators_enabled", False))
        homed_ok = bool(d.get("homed", False))

        if not is_tcp_connected:
            self.btn_enable_actuators.setEnabled(False)
            self.btn_home.setEnabled(False)
            self.btn_zero.setEnabled(False)
            self.btn_set_homing_vars.setEnabled(False)
            self.dpad_xy.setEnabled(False)
            self.dpad_zw.setEnabled(False)
            self.btn_jog_neg.setEnabled(False)
            self.btn_jog_pos.setEnabled(False)
            self.btn_move_abs.setEnabled(False)
            self.axis_tabs[ax].btn_start_wizard.setEnabled(False)
            return

        self.btn_enable_actuators.setEnabled(True)
        self.btn_home.setEnabled(actuators_ok)
        self.btn_zero.setEnabled(actuators_ok)
        self.btn_set_homing_vars.setEnabled(actuators_ok)

        self.axis_tabs[ax].btn_start_wizard.setEnabled(actuators_ok and self.axis_tabs[ax].calib_step == 0)

        can_move = actuators_ok and homed_ok
        self.dpad_xy.setEnabled(can_move)
        self.dpad_zw.setEnabled(can_move)
        self.btn_jog_neg.setEnabled(can_move)
        self.btn_jog_pos.setEnabled(can_move)
        self.btn_move_abs.setEnabled(can_move)

    def _axis_from_tab(self):
        idx = self.tabs.currentIndex()
        if idx < 0: return "x"
        return self.AXES[idx]

    def _next_seq(self):
        s = self.seq
        self.seq += 1
        return s

    def _send_obj(self, obj):
        if not self.worker or not self.worker.running:
            return
        try:
            self.worker.send_obj(obj)
        except Exception as e:
            self._log("WARN", f"Error enviando: {e}")

    def send_cmd(self, cmd, params):
        seq = self._next_seq()
        self._send_obj(build_message("cmd", f"msg-{seq}", seq, {"cmd": cmd, "params": params}))

    def send_axis_cmd(self, axis: str, cmd: str, params: dict):
        p = dict(params)
        p["axis"] = axis
        self.send_cmd(cmd, p)

    def _dpad_press_group(self, group: str, direction: str):
        if not self.dpad_xy.isEnabled():
            return

        if group == "xy":
            if direction == "up":
                axis, dir_cmd = "y", "forward"
            elif direction == "down":
                axis, dir_cmd = "y", "backward"
            elif direction == "right":
                axis, dir_cmd = "x", "forward"
            else:
                axis, dir_cmd = "x", "backward"

        self._dpad_active = True
        self._dpad_axis_group = group
        self._dpad_dir = direction
        self._is_moving = True

        self.send_axis_cmd(axis, "manual_start", {"dir": dir_cmd})

    def _dpad_release_direction(self, group: str, direction: str):
        self._dpad_active = False
        self._is_moving = False
        if group == "xy":
            if direction in ("up", "down"):
                self.send_axis_cmd("y", "manual_stop", {})
            elif direction in ("left", "right"):
                self.send_axis_cmd("x", "manual_stop", {})

    def _zw_vertical_press(self, action: str):
        if not self.dpad_zw.isEnabled():
            return
        
        mapping = {
            "z_up": ("z", "forward"),
            "z_down": ("z", "backward"),
            "w_up": ("w", "forward"),
            "w_down": ("w", "backward")
        }
        
        if action in mapping:
            axis, dir_cmd = mapping[action]
            self._dpad_active = True
            self._dpad_axis_group = "zw"
            self._is_moving = True
            self.send_axis_cmd(axis, "manual_start", {"dir": dir_cmd})

    def _zw_vertical_release(self, action: str):
        self._dpad_active = False
        self._is_moving = False
        mapping = {
            "z_up": "z", "z_down": "z",
            "w_up": "w", "w_down": "w"
        }
        if action in mapping:
            self.send_axis_cmd(mapping[action], "manual_stop", {})

    def mouseReleaseEvent(self, event):
        if self._dpad_active:
            self._dpad_release_group(self._dpad_axis_group)
        super().mouseReleaseEvent(event)

    def focusOutEvent(self, event):
        if self._dpad_active:
            self._dpad_stop_group(self._dpad_axis_group)
        super().focusOutEvent(event)

    def _dpad_release_group(self, group: str):
        self._dpad_active = False
        self._is_moving = False
        gaxes = ("x", "y") if group == "xy" else ("z", "w")
        for ax in gaxes:
            self.send_axis_cmd(ax, "manual_stop", {})

    def _dpad_stop_group(self, group: str):
        self._dpad_active = False
        self._is_moving = False
        if group == "xy" and self._dpad_dir:
            if self._dpad_dir in ("up", "down"):
                self.send_axis_cmd("y", "manual_stop", {})
            elif self._dpad_dir in ("left", "right"):
                self.send_axis_cmd("x", "manual_stop", {})
        elif group == "zw" and self._dpad_dir:
            if self._dpad_dir in ("z_up", "z_down"):
                self.send_axis_cmd("z", "manual_stop", {})
            elif self._dpad_dir in ("w_up", "w_down"):
                self.send_axis_cmd("w", "manual_stop", {})

    def on_connect(self):
        host = self.ed_host.text().strip()
        port = int(self.ed_port.value())

        self.worker = TcpWorker()
        self.worker.configure(host, port)
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)
        self.worker.tx_line.connect(self._on_tx_line)
        self.worker.parsed_msg.connect(self._on_parsed_msg)
        self.worker.error.connect(self._on_error)
        self.worker.start()

    def on_disconnect(self):
        self.status_timer.stop()
        self.pos_timer.stop()
        if self.worker:
            self.worker.stop()
            self.worker.wait(800)
            self.worker = None
        self._set_initial_disconnected_ui()

    def _on_connected(self):
        self._set_connected_ui(True)
        self._log("INFO", "Conexión TCP establecida")
        self.status_timer.start(50)
        self.pos_timer.start(20)
        self.poll_status()

    def _on_disconnected(self, reason):
        self._log("WARN", f"Desconectado: {reason}")
        self._set_initial_disconnected_ui()

    def _on_error(self, err):
        self._log("WARN", err)
        self._set_initial_disconnected_ui()

    def _set_connected_ui(self, ok):
        self.btn_connect.setEnabled(not ok)
        self.btn_disconnect.setEnabled(ok)
        if ok:
            self.lbl_conn_text.setText("CONECTADO")
            self._set_label_color_state(self.lbl_conn_text, "green")
        else:
            self._set_initial_disconnected_ui()
        self._update_controls_interlock()

    def send_home_axis(self):
        ax = self._axis_from_tab()
        self.send_axis_cmd(ax, "home_axis", {
            "backoff_steps": int(self.sp_main_backoff_steps.value()),
            "soft_offset_steps": int(self.sp_soft_offset_steps.value())
        })

    def send_zero_axis(self):
        ax = self._axis_from_tab()
        self.send_axis_cmd(ax, "set_zero_axis", {})

    def send_enable_actuators(self):
        if self.btn_enable_actuators.isChecked():
            self.send_cmd("enable_actuators", {})
            self.btn_enable_actuators.setEnabled(False)
            self.btn_enable_actuators.setText("Actuadores Activados")
            QTimer.singleShot(300, self._check_first_run_on_enable)
        self._update_controls_interlock()

    def _check_first_run_on_enable(self):
        if self.last_status and isinstance(self.last_status.get("axes"), dict):
            for ax_name, data in self.last_status["axes"].items():
                if data.get("first_run", False) and not self.setting_direction_in_progress:
                    self.setting_direction_in_progress = True
                    self.run_first_time_direction_wizard(ax_name)
                    break

    def run_first_time_direction_wizard(self, axis_name):
        QMessageBox.information(
            self,
            "Configuración Inicial",
            f"Eje [{axis_name.upper()}]: Motor se moverá 5000 pulsos"
        )
        self.send_axis_cmd(axis_name, "test_dir_pulse", {})

        reply = QMessageBox.question(
            self,
            "Validación",
            f"¿Eje [{axis_name.upper()}] se alejó del fin de carrera?",
            QMessageBox.Yes | QMessageBox.No
        )
        is_correct = (reply == QMessageBox.Yes)

        self.send_axis_cmd(axis_name, "confirm_axis_direction", {"is_correct": is_correct})
        
        if self.last_status and "axes" in self.last_status and axis_name in self.last_status["axes"]:
            self.last_status["axes"][axis_name]["first_run"] = False

        QMessageBox.information(self, "Guardado", f"Sentido guardado para eje [{axis_name.upper()}].")
        self.setting_direction_in_progress = False

    def on_motion_stop_clicked(self):
        ax = self._axis_from_tab()
        self.send_axis_cmd(ax, "manual_stop", {})
        self._log("INFO", "Motion Stop")

    def on_stop_clicked(self):
        self.send_cmd("emergency_stop", {})
        self.btn_enable_actuators.setEnabled(True)
        self.btn_enable_actuators.setChecked(False)
        self.btn_enable_actuators.setText("Activar actuadores")
        self._log("WARN", "E-STOP EJECUTADO")
        self._is_moving = False
        self._update_controls_interlock()

    def send_homing_vars(self):
        ax = self._axis_from_tab()
        tab = self.axis_tabs[ax]
        mt = float(self.sp_main_max_travel.value())
        
        if mt < 10.0:
            QMessageBox.warning(self, "Error", "Max Travel >= 10mm")
            return
        
        self.send_axis_cmd(ax, "set_homing_speed_axis", {
            "seek_us": int(self.sp_main_hseek.value()),
            "backoff_us": int(self.sp_main_hbo.value()),
        })
        
        self.send_axis_cmd(ax, "set_calibration_axis", {
            "steps_per_mm": float(tab.current_spm),
            "max_travel": mt,
        })

    def poll_status(self):
        if self.worker and self.worker.running:
            self.send_cmd("get_status", {})

    def _current_pos(self):
        ax = self._axis_from_tab()
        try:
            return float(self.last_status.get("axes", {}).get(ax, {}).get("pos", 0.0))
        except Exception:
            return 0.0

    def _check_target_inside_soft_limit(self, target):
        ax = self._axis_from_tab()
        d = self.last_status.get("axes", {}).get(ax, {}) if isinstance(self.last_status.get("axes"), dict) else {}
        try:
            max_travel = float(d.get("max_travel", self.sp_main_max_travel.value()))
            soft_offset_steps = float(d.get("soft_offset_steps", self.sp_soft_offset_steps.value()))
            spm = float(d.get("steps_per_mm", 568.0))
            soft_min = soft_offset_steps / spm if spm > 0 else 0.0
        except Exception:
            max_travel = float(self.sp_main_max_travel.value())
            soft_min = 0.0

        if target < soft_min or target > max_travel:
            self._log("WARN", f"Soft limit: objetivo {target:.3f} mm fuera de [{soft_min:.3f}, {max_travel:.3f}] mm")
            QMessageBox.warning(self, "Soft Limit", f"Objetivo {target:.3f} mm fuera de rango [{soft_min:.3f}, {max_travel:.3f}] mm")
            return False
        return True

    def send_move_abs(self):
        ax = self._axis_from_tab()
        target = float(self.sp_move_abs.value())
        if self._check_target_inside_soft_limit(target):
            self._is_moving = True
            self.send_axis_cmd(ax, "move_axis_abs", {"pos": target})

    def safe_send_relative(self, d):
        ax = self._axis_from_tab()
        target = self._current_pos() + d
        if self._check_target_inside_soft_limit(target):
            self._is_moving = True
            self.send_axis_cmd(ax, "move_axis_rel", {"delta": float(d)})

    def on_set_profile_and_speed(self, axis, tab):
        self.send_axis_cmd(axis, "set_manual_speed_axis", {
            "delay_us": int(tab.sp_manual_us.value()),
        })

    def _update_position_smoothly(self):
        """Actualiza la posición de forma suave sin esperar status"""
        ax = self._axis_from_tab()
        d = self.last_status.get("axes", {}).get(ax, {}) if isinstance(self.last_status.get("axes"), dict) else {}

        if not d:
            return

        if d.get("is_moving", False) and d.get("move_dir", "none") != "none":
            try:
                pos = float(d.get("pos", 0.0))
                spm = float(d.get("steps_per_mm", 568.0))

                if spm > 0:
                    if d.get("move_dir") == "forward":
                        pos += (1.0 / spm)
                    elif d.get("move_dir") == "backward":
                        pos -= (1.0 / spm)

                    self.lbl_pos.setText(f"{pos:.3f} mm")

            except Exception:
                pass

    def _on_tab_changed(self, idx):
        ax = self._axis_from_tab().upper()
        self.lbl_axis_active.setText(ax)
        
        ax_key = self._axis_from_tab()
        d = self.last_status.get("axes", {}).get(ax_key, {}) if isinstance(self.last_status.get("axes"), dict) else {}
        if d:
            if "max_travel" in d:
                self.sp_main_max_travel.setValue(float(d["max_travel"]))
            if "home_seek_us" in d:
                self.sp_main_hseek.setValue(int(d["home_seek_us"]))
            if "home_backoff_us" in d:
                self.sp_main_hbo.setValue(int(d["home_backoff_us"]))
            if "steps_per_mm" in d:
                self.axis_tabs[ax_key].current_spm = float(d["steps_per_mm"])
            if "backoff_steps" in d:
                self.sp_main_backoff_steps.setValue(int(d["backoff_steps"]))
            if "soft_offset_steps" in d:
                self.sp_soft_offset_steps.setValue(int(d["soft_offset_steps"]))
            if "last_calibration" in d:
                self.lbl_calib_date.setText(str(d["last_calibration"]))
            if "manual_us" in d:
                self.sp_dpad_speed.setValue(int(d["manual_us"]))

        self._update_controls_interlock()
        self._update_status_ui(self.last_status or {})

    def _set_machine_state_label(self, state: str):
        txt = (state or "-").upper()
        self.lbl_machine_state_v.setText(txt)
        s = txt.lower()
        if s == "idle":
            c = "green"
        elif s in ("running", "manual"):
            c = "blue"
        elif s in ("homing", "calibrating"):
            c = "yellow"
        elif s == "alarm":
            c = "red"
        else:
            c = "default"
        self._set_label_color_state(self.lbl_machine_state_v, c)

    def _on_status_update(self, payload: dict):
        self._update_status_ui(payload)

    def _update_status_ui(self, payload: dict):
        if not payload:
            return

        with self.status_lock:
            self._set_machine_state_label(str(payload.get("machine_state", "-")))

            act_ok = bool(payload.get("actuators_enabled", False))
            self.lbl_actuators_text.setText("HABILITADOS" if act_ok else "DESHABILITADOS")
            self._set_label_color_state(self.lbl_actuators_text, "green" if act_ok else "red")

            ax = self._axis_from_tab()
            d = payload.get("axes", {}).get(ax, {}) if isinstance(payload.get("axes"), dict) else {}

            try:
                pos = float(d.get("pos", 0.0))
                target_pos = float(d.get("target_pos", pos))
                woff = float(d.get("work_offset", 0.0))

                if "step_count" in d:
                    self.axis_tabs[ax].set_steps_count(int(d["step_count"]))

                is_moving = bool(d.get("is_moving", False))
                self._is_moving = is_moving

            except Exception:
                pos, target_pos, woff = 0.0, 0.0, 0.0

            self.lbl_pos.setText(f"{pos:.3f} mm")
            self.lbl_woff.setText(f"{woff:.3f} mm")
            self.lbl_target_pos.setText(f"{target_pos:.3f} mm")

            homed_ok = bool(d.get("homed", False))
            self.lbl_homed_text.setText("HOMED" if homed_ok else "NO HOMED")
            self._set_label_color_state(self.lbl_homed_text, "green" if homed_ok else "red")

            if "last_calibration" in d and d["last_calibration"]:
                self.lbl_calib_date.setText(str(d["last_calibration"]))

            is_moving = bool(d.get("is_moving", False))
            self.lbl_moving_text.setText("EN MOV" if is_moving else "DETENIDO")
            self._set_label_color_state(self.lbl_moving_text, "blue" if is_moving else "green")

            move_dir = str(d.get("move_dir", "none")).upper()
            self.lbl_dir_text.setText(move_dir)

            err = str(d.get("last_error", "") or "").strip()
            self.lbl_last_error.setText(err if err else "OK")

            self._update_controls_interlock()

    def _log(self, level, text):
        allowed = {"SEND", "STATUS", "INFO", "WARN"}
        if level not in allowed:
            return
        colors = {"SEND": "#58a6ff", "STATUS": "#7ee787", "INFO": "#c9d1d9", "WARN": "#f2cc60"}
        self.txt_log.setTextColor(QColor(colors.get(level, "#c9d1d9")))
        self.txt_log.append(f"[{now_str()}] [{level}] {text}")
        
        if self._auto_scroll_terminal:
            self.txt_log.moveCursor(QTextCursor.End)

    def _on_tx_line(self, line):
        self._log("SEND", line)

    def _on_parsed_msg(self, msg):
        try:
            mtype = msg.get("type", "")
            payload = msg.get("payload", {}) or {}

            if mtype == "nack":
                reason = payload.get("reason", "error")
                self._log("WARN", f"NACK: {reason}")
                return

            if mtype in ("ack", "status"):
                with self.status_lock:
                    self.last_status = payload
                self.update_status_signal.emit(payload)
                self._log("STATUS", f"state={payload.get('machine_state','-')}")
        except Exception as e:
            self._log("WARN", f"Error: {e}")

    def closeEvent(self, event):
        self.status_timer.stop()
        self.pos_timer.stop()
        self._dpad_stop_group("xy")
        self._dpad_stop_group("zw")
        try:
            if self.worker:
                self.worker.stop()
                self.worker.wait(800)
        except Exception:
            pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(1280, 750)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()