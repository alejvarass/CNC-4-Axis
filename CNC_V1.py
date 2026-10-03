import sys
import socket
from datetime import datetime
import math
import json
import cv2

from PySide6.QtCore import Qt, QThread, Signal, QPointF, QRectF, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPainterPath, QImage
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QSpinBox, QDoubleSpinBox, QGroupBox,
    QMessageBox, QTabWidget, QCheckBox, QSlider, QComboBox, QFormLayout,
    QDialog, QListWidget, QFileDialog
)

def now_str():
    return datetime.now().strftime("%H:%M:%S")


class ToggleSwitch(QWidget):
    toggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(54, 22)
        self._checked = False

    def isChecked(self):
        return self._checked

    def setChecked(self, checked: bool):
        if self._checked != checked:
            self._checked = checked
            self.toggled.emit(self._checked)
            self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.setChecked(not self._checked)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        bg_color = QColor("#238636") if self._checked else QColor("#30363d")
        p.setPen(Qt.NoPen)
        p.setBrush(bg_color)
        r = self.rect()
        p.drawRoundedRect(r, 11, 11)

        thumb_x = r.width() - 20 if self._checked else 2
        p.setBrush(QColor("#ffffff"))
        p.drawEllipse(thumb_x, 2, 18, 18)
        p.end()


class CameraMeasurementWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 180)
        self.setMouseTracking(True)

        self.cap = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_frame)

        self.raw_frame = None
        self.grid_enabled = True
        self.grid_size = 40
        self.show_crosshair = True
        self.measure_mode = True

        self.zoom_factor = 1.0
        self.px_per_mm = 10.0

        self.measuring = False
        self.p1 = None
        self.p2 = None
        self.measurement_lines = []

    def start_camera(self, index=0):
        self.stop_camera()
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) if sys.platform.startswith("win") else cv2.VideoCapture(index)
        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 10000)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 10000)
            self.timer.start(30)
            return True
        return False

    def stop_camera(self):
        if self.timer.isActive():
            self.timer.stop()
        if self.cap:
            self.cap.release()
            self.cap = None
        self.raw_frame = None
        self.update()

    def set_grid_enabled(self, enabled: bool):
        self.grid_enabled = bool(enabled)
        self.update()

    def set_grid_size(self, size: int):
        self.grid_size = max(5, int(size))
        self.update()

    def set_zoom(self, val_pct: int):
        self.zoom_factor = max(1.0, float(val_pct) / 100.0)
        self.update()

    def set_px_per_mm(self, px_for_10mm: float):
        if px_for_10mm > 0:
            self.px_per_mm = px_for_10mm / 10.0
            self.update()

    def clear_measurements(self):
        self.measurement_lines.clear()
        self.p1 = None
        self.p2 = None
        self.update()

    def _update_frame(self):
        if not self.cap or not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if ret:
            self.raw_frame = frame
            self.update()

    def _get_cropped_frame(self):
        if self.raw_frame is None:
            return None
        h, w = self.raw_frame.shape[:2]
        if abs(self.zoom_factor - 1.0) < 0.01:
            return self.raw_frame

        crop_w = int(w / self.zoom_factor)
        crop_h = int(h / self.zoom_factor)
        x1 = (w - crop_w) // 2
        y1 = (h - crop_h) // 2
        return self.raw_frame[y1:y1 + crop_h, x1:x1 + crop_w]

    def _img_to_widget_scale(self, frame):
        if frame is None:
            return 1.0, 0, 0, 0, 0
        h_img, w_img = frame.shape[:2]
        w_w, h_w = self.width(), self.height()
        scale = min(w_w / w_img, h_w / h_img)
        disp_w = int(w_img * scale)
        disp_h = int(h_img * scale)
        off_x = (w_w - disp_w) / 2.0
        off_y = (h_w - disp_h) / 2.0
        return scale, off_x, off_y, disp_w, disp_h

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton and self.measure_mode:
            self.measuring = True
            self.p1 = e.position()
            self.p2 = e.position()
            self.update()

    def mouseMoveEvent(self, e):
        if self.measuring:
            self.p2 = e.position()
            self.update()

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton and self.measuring:
            self.measuring = False
            self.p2 = e.position()
            if self.p1 and (self.p1 - self.p2).manhattanLength() > 4:
                self.measurement_lines.append((self.p1, self.p2))
            self.p1 = None
            self.p2 = None
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#080c10"))

        frame = self._get_cropped_frame()
        if frame is None:
            p.setPen(QColor("#6e7681"))
            p.setFont(QFont("Segoe UI", 10, QFont.Bold))
            p.drawText(self.rect(), Qt.AlignCenter, "[ CÁMARA DESCONECTADA / SIN SEÑAL ]")
            p.end()
            return

        h_img, w_img, ch = frame.shape
        scale, off_x, off_y, disp_w, disp_h = self._img_to_widget_scale(frame)
        target_rect = QRectF(off_x, off_y, disp_w, disp_h)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb_frame.data, w_img, h_img, ch * w_img, QImage.Format_RGB888)
        p.drawImage(target_rect, qimg)

        if self.grid_enabled and self.grid_size > 4:
            p.save()
            p.setClipRect(target_rect)
            p.setPen(QPen(QColor(57, 255, 20, 160), 1, Qt.DashLine))
            step = float(self.grid_size) * scale

            cx = target_rect.center().x()
            x = cx
            while x <= target_rect.right():
                p.drawLine(QPointF(x, target_rect.top()), QPointF(x, target_rect.bottom()))
                x += step
            x = cx - step
            while x >= target_rect.left():
                p.drawLine(QPointF(x, target_rect.top()), QPointF(x, target_rect.bottom()))
                x -= step

            cy = target_rect.center().y()
            y = cy
            while y <= target_rect.bottom():
                p.drawLine(QPointF(target_rect.left(), y), QPointF(target_rect.right(), y))
                y += step
            y = cy - step
            while y >= target_rect.top():
                p.drawLine(QPointF(target_rect.left(), y), QPointF(target_rect.right(), y))
                y -= step
            p.restore()

        if self.show_crosshair:
            cx = target_rect.center().x()
            cy = target_rect.center().y()
            p.setPen(QPen(QColor("#39ff14"), 1))
            p.drawLine(QPointF(cx - 15, cy), QPointF(cx + 15, cy))
            p.drawLine(QPointF(cx, cy - 15), QPointF(cx, cy + 15))
            p.drawEllipse(QPointF(cx, cy), 8, 8)

        all_lines = list(self.measurement_lines)
        if self.measuring and self.p1 and self.p2:
            all_lines.append((self.p1, self.p2))

        def draw_cross_marker(painter, pt, size=6):
            painter.drawLine(QPointF(pt.x() - size, pt.y()), QPointF(pt.x() + size, pt.y()))
            painter.drawLine(QPointF(pt.x(), pt.y() - size), QPointF(pt.x(), pt.y() + size))

        for pt1, pt2 in all_lines:
            p.setPen(QPen(QColor("#ff0055"), 2))
            p.drawLine(pt1, pt2)

            p.setPen(QPen(QColor("#00f0ff"), 2))
            draw_cross_marker(p, pt1, 6)
            draw_cross_marker(p, pt2, 6)

            dx_scr = pt2.x() - pt1.x()
            dy_scr = pt2.y() - pt1.y()
            dist_screen_px = math.hypot(dx_scr, dy_scr)
            dist_frame_px = (dist_screen_px / (scale if scale > 0 else 1.0)) / self.zoom_factor
            dist_mm = dist_frame_px / (self.px_per_mm if self.px_per_mm > 0 else 1.0)

            mid = (pt1 + pt2) / 2.0
            p.setFont(QFont("Consolas", 10, QFont.Bold))
            tag_txt = f"{dist_mm:.3f} mm"

            text_rect = QRectF(mid.x() - 50, mid.y() - 10, 100, 20)
            p.setPen(QColor("#000000"))
            p.drawText(text_rect.translated(1, 1), Qt.AlignCenter, tag_txt)
            p.setPen(QColor("#00f0ff"))
            p.drawText(text_rect, Qt.AlignCenter, tag_txt)

        p.end()


class DPadPainter(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)
    stopPressed = Signal()

    def __init__(self, labels=None, parent=None):
        super().__init__(parent)
        self.setFixedSize(190, 190)
        self._pressed = False
        self._active = None
        self.setMouseTracking(True)
        self.labels = labels or {"up": "↑", "down": "↓", "left": "←", "right": "→"}

    def _geom(self):
        w, h = self.width(), self.height()
        c = QPointF(w / 2, h / 2)
        outer = min(w, h) * 0.46
        inner = min(w, h) * 0.23
        return c, outer, inner, inner

    def _hit(self, pos):
        c, outer, inner, stop_r = self._geom()
        x = pos.x() - c.x()
        y = pos.y() - c.y()
        r = math.hypot(x, y)
        if r <= stop_r: return "stop"
        if r > outer: return None
        ang = math.degrees(math.atan2(-y, x))
        if ang < 0: ang += 360
        if 45 <= ang < 135: return "up"
        if 135 <= ang < 225: return "left"
        if 225 <= ang < 315: return "down"
        return "right"

    def mousePressEvent(self, e):
        if not self.isEnabled() or e.button() != Qt.LeftButton: return
        zone = self._hit(e.position())
        if zone is None: return
        self._pressed = True
        self._active = zone
        if zone == "stop": self.stopPressed.emit()
        else: self.directionPressed.emit(zone)
        self.update()

    def mouseMoveEvent(self, e):
        if not self.isEnabled() or not self._pressed: return
        zone = self._hit(e.position())
        if zone in ("up", "down", "left", "right", "stop") and zone != self._active:
            old_zone = self._active
            self._active = zone
            if old_zone and old_zone != "stop": self.directionReleased.emit(old_zone)
            if zone == "stop": self.stopPressed.emit()
            else: self.directionPressed.emit(zone)
            self.update()

    def mouseReleaseEvent(self, e):
        if self._pressed:
            old = self._active
            self._pressed = False
            self._active = None
            if old and old != "stop": self.directionReleased.emit(old)
            self.update()

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        c, outer, inner, stop_r = self._geom()
        p.fillRect(self.rect(), QColor("#161b22"))
        p.setPen(QPen(QColor("#2f4953"), 1)); p.setBrush(QColor("#0f1a20"))
        p.drawEllipse(c, outer, outer)
        gap = 3.0

        def draw_sector(start_deg, span_deg, key):
            outer_rect = QRectF(c.x() - outer, c.y() - outer, outer * 2, outer * 2)
            inner_rect = QRectF(c.x() - inner, c.y() - inner, inner * 2, inner * 2)
            path = QPainterPath()
            path.arcMoveTo(outer_rect, start_deg + gap / 2.0)
            path.arcTo(outer_rect, start_deg + gap / 2.0, span_deg - gap)
            path.arcTo(inner_rect, start_deg + span_deg - gap / 2.0, -(span_deg - gap))
            path.closeSubpath()
            p.setPen(QPen(QColor("#2f4953"), 1))
            p.setBrush(QColor("#1f4958") if self._active == key else (QColor("#12232c") if self.isEnabled() else QColor("#1c2128")))
            p.drawPath(path)

        draw_sector(45, 90, "up"); draw_sector(135, 90, "left")
        draw_sector(225, 90, "down"); draw_sector(315, 90, "right")

        p.setPen(QPen(QColor("#b02a2a"), 2))
        p.setBrush(QColor("#a42424") if self._active == "stop" else QColor("#7d1b1b"))
        p.drawEllipse(c, stop_r, stop_r)
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", max(9, int(inner * 0.35)), QFont.Bold))
        p.drawText(QRectF(c.x() - 18, c.y() - outer + 6, 36, 20), Qt.AlignCenter, self.labels.get("up", "↑"))
        p.drawText(QRectF(c.x() - outer + 4, c.y() - 10, 36, 20), Qt.AlignCenter, self.labels.get("left", "←"))
        p.drawText(QRectF(c.x() + outer - 40, c.y() - 10, 36, 20), Qt.AlignCenter, self.labels.get("right", "→"))
        p.drawText(QRectF(c.x() - 18, c.y() + outer - 26, 36, 20), Qt.AlignCenter, self.labels.get("down", "↓"))
        p.setFont(QFont("Segoe UI", max(8, int(stop_r * 0.32)), QFont.Bold))
        p.drawText(QRectF(c.x() - stop_r, c.y() - stop_r, stop_r * 2, stop_r * 2), Qt.AlignCenter, "E STOP")
        p.end()


class VerticalZWControl(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(145, 190)
        self._pressed = False
        self._active = None
        self.setMouseTracking(True)

    def _get_outer_oval_rect(self):
        w, h = self.width(), self.height()
        oval_w = min(w * 0.84, 120); oval_h = min(h * 0.96, 184)
        return QRectF((w - oval_w) / 2, (h - oval_h) / 2, oval_w, oval_h)

    def _get_inner_oval_rect(self, outer_r, gap):
        h_sec = (outer_r.height() - gap * 3) / 4.0
        top_w = outer_r.top() + h_sec + gap
        return QRectF(outer_r.left() + outer_r.width() * 0.08, top_w, outer_r.width() * 0.84, (h_sec * 2) + gap)

    def _hit(self, pos):
        outer_r = self._get_outer_oval_rect()
        if not outer_r.contains(pos): return None
        gap = 4.0; inner_r = self._get_inner_oval_rect(outer_r, gap)
        if inner_r.contains(pos): return "w_up" if pos.y() < inner_r.center().y() else "w_down"
        return "z_up" if pos.y() < outer_r.center().y() else "z_down"

    def mousePressEvent(self, e):
        if not self.isEnabled() or e.button() != Qt.LeftButton: return
        zone = self._hit(e.position())
        if zone:
            self._pressed = True; self._active = zone
            self.directionPressed.emit(zone)
            self.update()

    def mouseMoveEvent(self, e):
        if not self.isEnabled() or not self._pressed: return
        zone = self._hit(e.position())
        if zone != self._active:
            if self._active: self.directionReleased.emit(self._active)
            self._active = zone
            if zone: self.directionPressed.emit(zone)
            self.update()

    def mouseReleaseEvent(self, e):
        if self._pressed:
            if self._active: self.directionReleased.emit(self._active)
            self._pressed = False; self._active = None
            self.update()

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#161b22"))
        outer_r = self._get_outer_oval_rect(); gap = 4.0
        inner_r = self._get_inner_oval_rect(outer_r, gap)

        bg_out = QPainterPath(); bg_out.addRoundedRect(outer_r, outer_r.width() / 2, outer_r.width() / 2)
        bg_in = QPainterPath(); bg_in.addRoundedRect(inner_r, inner_r.width() / 2, inner_r.width() / 2)
        p.setPen(QPen(QColor("#2f4953"), 1)); p.setBrush(QColor("#0f1a20")); p.drawPath(bg_out)

        color_idle = QColor("#12232c") if self.isEnabled() else QColor("#1c2128")
        color_act = QColor("#1f4958")
        h_sec = (outer_r.height() - gap * 3) / 4.0

        sec_z_up = QRectF(outer_r.left(), outer_r.top(), outer_r.width(), h_sec)
        path_z_up = QPainterPath(); path_z_up.addRect(sec_z_up)
        p.setBrush(color_act if self._active == "z_up" else color_idle); p.drawPath(path_z_up.intersected(bg_out))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58")); p.setFont(QFont("Segoe UI", 10, QFont.Bold))
        p.drawText(sec_z_up, Qt.AlignCenter, "Z+")

        sec_z_down = QRectF(outer_r.left(), outer_r.top() + (h_sec + gap) * 3, outer_r.width(), h_sec)
        path_z_down = QPainterPath(); path_z_down.addRect(sec_z_down)
        p.setBrush(color_act if self._active == "z_down" else color_idle); p.drawPath(path_z_down.intersected(bg_out))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.drawText(sec_z_down, Qt.AlignCenter, "Z-")

        p.setPen(QPen(QColor("#00f0ff") if self.isEnabled() else QColor("#2f4953"), 1, Qt.DashLine))
        p.setBrush(QColor("#0d181e")); p.drawPath(bg_in)

        half_inner = (inner_r.height() - gap) / 2.0
        sec_w_up = QRectF(inner_r.left(), inner_r.top(), inner_r.width(), half_inner)
        path_w_up = QPainterPath(); path_w_up.addRect(sec_w_up)
        p.setBrush(color_act if self._active == "w_up" else color_idle); p.drawPath(path_w_up.intersected(bg_in))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58")); p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.drawText(sec_w_up, Qt.AlignCenter, "W+")

        sec_w_down = QRectF(inner_r.left(), inner_r.top() + half_inner + gap, inner_r.width(), half_inner)
        path_w_down = QPainterPath(); path_w_down.addRect(sec_w_down)
        p.setBrush(color_act if self._active == "w_down" else color_idle); p.drawPath(path_w_down.intersected(bg_in))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
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
        self._rx_buffer = b""

    def configure(self, host: str, port: int):
        self.host = host; self.port = port

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.05)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.connect((self.host, self.port))
            self.running = True
            self.connected.emit()

            while self.running:
                try:
                    chunk = self.sock.recv(2048)
                    if not chunk:
                        self.running = False
                        self.disconnected.emit("Servidor cerró conexión")
                        break
                    self._rx_buffer += chunk
                    while b"\n" in self._rx_buffer:
                        line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                        line_str = line.decode("utf-8", errors="replace").strip()
                        if line_str: self._parse_line(line_str)
                except socket.timeout: continue
                except Exception as e:
                    self.running = False
                    self.disconnected.emit(f"RX error: {e}")
                    break
        except Exception as e: 
            self.running = False
            self.error.emit(f"No se pudo conectar: {e}")
        finally: 
            self._close_socket()

    def _parse_line(self, line: str):
        if line.startswith("<") and line.endswith(">"):
            self.parsed_msg.emit({"type": "grbl_status", "payload": line})
            return

        parts = line.split("|")
        tag = parts[0]
        if tag == "ST" and len(parts) >= 7:
            payload = {
                "machine_state": parts[1],
                "actuators_enabled": (parts[2] == "1"),
                "axes": {}
            }
            axes_names = ["x", "y", "z", "w"]
            for i, ax_data in enumerate(parts[3:7]):
                f = ax_data.split(",")
                if len(f) >= 24:
                    payload["axes"][axes_names[i]] = {
                        "pos": float(f[0]), "target_pos": float(f[1]), "step_count": int(f[2]),
                        "homed": (f[3] == "1"), "calibrated": (f[4] == "1"), "first_run": (f[5] == "1"),
                        "dir_forward_level": int(f[6]), "is_moving": (f[7] == "1"), "move_dir": f[8],
                        "steps_per_mm": float(f[9]), "max_travel": float(f[10]),
                        "backoff_steps": int(f[11]), "soft_offset_steps": int(f[12]), "use_scurve": (f[13] == "1"),
                        "manual_us": int(f[14]), "jog_us": int(f[15]), "home_seek_us": int(f[16]), "home_backoff_us": int(f[17]),
                        "last_calibration": f[18] if f[18] != "None" else "",
                        "scurve_profile": {
                            "start_mm_s": float(f[19]), "cruise_mm_s": float(f[20]), "end_mm_s": float(f[21]),
                            "ramp_ratio": float(f[22])
                        },
                        "last_error": f[23] if f[23] != "None" else ""
                    }
            self.parsed_msg.emit({"type": "status", "payload": payload})
        elif tag == "ACK":
            self.parsed_msg.emit({"type": "ack", "payload": {"cmd": parts[1] if len(parts)>1 else ""}})

    def send_line(self, line: str):
        if not self.sock: return
        payload = (line.strip() + "\n").encode("utf-8")
        try:
            self.sock.sendall(payload)
            self.tx_line.emit(line.strip())
        except Exception: pass

    def stop(self):
        self.running = False
        self._close_socket()

    def _close_socket(self):
        if self.sock:
            try: 
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception: pass
            self.sock = None


class CalibrationDialog(QDialog):
    def __init__(self, axis_tab, parent=None):
        super().__init__(parent)
        self.axis_tab = axis_tab
        self.setWindowTitle(f"Calibración 2 Pasadas - {self.axis_tab.axis.upper()}")
        self.setMinimumWidth(380)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(6)

        gb = QGroupBox("Asistente de Calibración")
        form = QFormLayout(gb)
        form.setContentsMargins(6, 8, 6, 6)
        form.setSpacing(5)

        self.lbl_last_date = QLabel(self.axis_tab.lbl_last_calib_date.text())
        self.lbl_last_date.setStyleSheet("color: #8b949e; font-weight: bold;")

        self.lbl_calib_instructions = QLabel(self.axis_tab.lbl_calib_instructions.text())
        self.lbl_calib_instructions.setStyleSheet("color: #e6edf3; font-size: 13px;")
        self.lbl_calib_instructions.setWordWrap(True)

        self.lbl_pasos = QLabel(self.axis_tab.lbl_pasos_acumulados.text())
        self.lbl_pasos.setStyleSheet("font-weight: bold; color: #58a6ff; font-size: 14px;")

        self.btn_calc = QPushButton("Calcular")
        self.btn_calc.setEnabled(self.axis_tab.btn_calculate_preview.isEnabled())

        row_steps = QHBoxLayout(); row_steps.setSpacing(4)
        row_steps.addWidget(self.lbl_pasos, 1)
        row_steps.addWidget(self.btn_calc, 1)

        self.lbl_calc_spm = QLabel(self.axis_tab.lbl_calculated_spm.text())
        self.lbl_calc_spm.setStyleSheet("font-weight: bold; color: #00f0ff; font-size: 14px;")

        self.sp_dist = QDoubleSpinBox()
        self.sp_dist.setRange(0.0, 5000.0); self.sp_dist.setDecimals(3)
        self.sp_dist.setValue(self.axis_tab.sp_distancia.value())
        self.sp_dist.setEnabled(self.axis_tab.sp_distancia.isEnabled())

        self.btn_start = QPushButton(self.axis_tab.btn_start_wizard.text())
        self.btn_start.setEnabled(self.axis_tab.btn_start_wizard.isEnabled())

        self.btn_confirm = QPushButton(self.axis_tab.btn_confirm_pass.text())
        self.btn_confirm.setEnabled(self.axis_tab.btn_confirm_pass.isEnabled())

        form.addRow("Última:", self.lbl_last_date)
        form.addRow("Estado:", self.lbl_calib_instructions)
        form.addRow("Pasos ESP32:", row_steps)
        form.addRow("Calculado:", self.lbl_calc_spm)
        form.addRow("Dist. (mm):", self.sp_dist)
        form.addRow(self.btn_start)
        form.addRow(self.btn_confirm)

        lay.addWidget(gb)

        self.btn_start.clicked.connect(self._on_start)
        self.btn_confirm.clicked.connect(self._on_confirm)
        self.btn_calc.clicked.connect(self._on_calc)
        self.sp_dist.valueChanged.connect(self._on_dist_changed)

    def _on_dist_changed(self, v):
        self.axis_tab.sp_distancia.setValue(v)

    def _on_start(self):
        self.axis_tab._start_calibration_wizard()
        self.sync_ui()

    def _on_confirm(self):
        self.axis_tab.sp_distancia.setValue(self.sp_dist.value())
        self.axis_tab._confirm_pass_measurement()
        self.sync_ui()

    def _on_calc(self):
        self.axis_tab.sp_distancia.setValue(self.sp_dist.value())
        self.axis_tab._calculate_preview()
        self.sync_ui()

    def sync_ui(self):
        self.lbl_last_date.setText(self.axis_tab.lbl_last_calib_date.text())
        self.lbl_calib_instructions.setText(self.axis_tab.lbl_calib_instructions.text())
        self.lbl_pasos.setText(self.axis_tab.lbl_pasos_acumulados.text())
        self.lbl_calc_spm.setText(self.axis_tab.lbl_calculated_spm.text())
        self.sp_dist.setEnabled(self.axis_tab.sp_distancia.isEnabled())
        self.sp_dist.setValue(self.axis_tab.sp_distancia.value())
        self.btn_start.setText(self.axis_tab.btn_start_wizard.text())
        self.btn_start.setEnabled(self.axis_tab.btn_start_wizard.isEnabled())
        self.btn_confirm.setText(self.axis_tab.btn_confirm_pass.text())
        self.btn_confirm.setEnabled(self.axis_tab.btn_confirm_pass.isEnabled())
        self.btn_calc.setEnabled(self.axis_tab.btn_calculate_preview.isEnabled())


class AxisBlock(QGroupBox):
    def __init__(self, axis_name: str, main_window):
        super().__init__()
        self.axis = axis_name.lower()
        self.main_window = main_window
        self.setProperty("moving", False)
        self.setProperty("homed", False)
        
        self.last_start_pos = 0.0
        self.was_moving = False
        self.is_homed = False

        main_lay = QVBoxLayout(self)
        main_lay.setContentsMargins(6, 6, 6, 6)
        main_lay.setSpacing(3)

        row1 = QHBoxLayout()
        row1.setContentsMargins(0, 0, 0, 0)
        row1.setSpacing(4)

        self.lbl_axis_tag = QLabel(self.axis.upper())
        self.lbl_axis_tag.setStyleSheet("color: #ffb347; font-size: 24px; font-weight: bold;")
        self.lbl_axis_tag.setFixedWidth(30)

        pos_box = QHBoxLayout()
        pos_box.setAlignment(Qt.AlignCenter)
        pos_box.setSpacing(4)
        self.lbl_pos_tag = QLabel("Pos:")
        self.lbl_pos_tag.setStyleSheet("font-size: 24px; font-weight: bold; color: #f2cc60;")
        self.lbl_pos_num = QLabel("0.000 mm")
        self.lbl_pos_num.setStyleSheet("font-size: 24px; font-weight: bold; color: #00f0ff;")
        pos_box.addWidget(self.lbl_pos_tag)
        pos_box.addWidget(self.lbl_pos_num)

        self.lbl_homing = QLabel("NO HOME")
        self.lbl_homing.setAlignment(Qt.AlignCenter)
        self.lbl_homing.setFixedWidth(75)
        self.lbl_homing.setStyleSheet("background-color: #551a1a; color: #ff7b72; font-weight: bold; border-radius: 3px; padding: 2px; font-size: 12px;")

        row1.addWidget(self.lbl_axis_tag)
        row1.addLayout(pos_box, 1)
        row1.addWidget(self.lbl_homing)
        main_lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.setContentsMargins(0, 0, 0, 0)
        row2.setSpacing(6)
        row2.setAlignment(Qt.AlignCenter)

        self.lbl_lpos_title = QLabel("L.Pos:")
        self.lbl_lpos_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.lbl_lpos_val = QLabel("0.000")
        self.lbl_lpos_val.setStyleSheet("color: #d2a8ff; font-weight: bold; font-size: 14px;")

        self.lbl_woff_title = QLabel("WOff:")
        self.lbl_woff_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.lbl_woff = QLabel("0.000")
        self.lbl_woff.setStyleSheet("color: #7ee787; font-weight: bold; font-size: 14px;")

        self.lbl_dir_title = QLabel("Dir:")
        self.lbl_dir_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.lbl_dir = QLabel("STOP")
        self.lbl_dir.setStyleSheet("font-weight: bold; color: #8b949e; font-size: 14px;")

        row2.addWidget(self.lbl_lpos_title)
        row2.addWidget(self.lbl_lpos_val)
        row2.addSpacing(10)
        row2.addWidget(self.lbl_woff_title)
        row2.addWidget(self.lbl_woff)
        row2.addSpacing(10)
        row2.addWidget(self.lbl_dir_title)
        row2.addWidget(self.lbl_dir)
        main_lay.addLayout(row2)

        row3 = QHBoxLayout()
        row3.setContentsMargins(0, 0, 0, 0)
        row3.setSpacing(4)
        row3.setAlignment(Qt.AlignCenter)

        self.lbl_abs_title = QLabel("Abs:")
        self.lbl_abs_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.sp_abs = QDoubleSpinBox()
        self.sp_abs.setRange(-5000.0, 5000.0); self.sp_abs.setDecimals(3); self.sp_abs.setFixedWidth(72)
        self.sp_abs.setStyleSheet("font-size: 13px;")
        self.btn_abs = QPushButton("Abs")
        self.btn_abs.setFixedWidth(42)

        self.lbl_rel_title = QLabel("Rel:")
        self.lbl_rel_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.sp_rel = QDoubleSpinBox()
        self.sp_rel.setRange(0.001, 500.0); self.sp_rel.setDecimals(3); self.sp_rel.setValue(10.0); self.sp_rel.setFixedWidth(66)
        self.sp_rel.setStyleSheet("font-size: 13px;")
        self.btn_rel_neg = QPushButton("- Rel")
        self.btn_rel_neg.setFixedWidth(48)
        self.btn_rel_pos = QPushButton("+ Rel")
        self.btn_rel_pos.setFixedWidth(48)

        self.btn_bkpos = QPushButton("BKPos")
        self.btn_bkpos.setFixedWidth(54)
        self.btn_bkpos.setToolTip("Regresa al eje a la posición de partida (L.Pos) del último movimiento")

        row3.addWidget(self.lbl_abs_title)
        row3.addWidget(self.sp_abs)
        row3.addWidget(self.btn_abs)
        row3.addSpacing(4)
        row3.addWidget(self.lbl_rel_title)
        row3.addWidget(self.sp_rel)
        row3.addWidget(self.btn_rel_neg)
        row3.addWidget(self.btn_rel_pos)
        row3.addSpacing(4)
        row3.addWidget(self.btn_bkpos)
        main_lay.addLayout(row3)

        row4 = QHBoxLayout()
        row4.setContentsMargins(0, 0, 0, 0)
        row4.setSpacing(5)
        row4.setAlignment(Qt.AlignCenter)

        self.chk_scurve = QCheckBox("Curva S")
        self.chk_scurve.setChecked(True)
        self.chk_scurve.setToolTip("Activado = Aceleración Curva en S. Desactivado = Velocidad Constante")
        self.chk_scurve.setStyleSheet("font-size: 13px; font-weight: bold; color: #7ee787;")

        self.btn_zero = QPushButton("Set Zero")
        self.btn_zero.setFixedWidth(70)
        self.btn_home = QPushButton("HOMING")
        self.btn_home.setFixedWidth(75)

        self.lbl_err_title = QLabel("Error:")
        self.lbl_err_title.setStyleSheet("font-size: 13px; font-weight: bold;")
        self.lbl_error = QLabel("Sin error")
        self.lbl_error.setStyleSheet("color: #ff7b72; font-size: 13px; font-weight: bold;")

        row4.addWidget(self.chk_scurve)
        row4.addWidget(self.btn_zero)
        row4.addWidget(self.btn_home)
        row4.addSpacing(4)
        row4.addWidget(self.lbl_err_title)
        row4.addWidget(self.lbl_error)
        main_lay.addLayout(row4)

        self.btn_abs.clicked.connect(self._on_move_abs)
        self.btn_rel_neg.clicked.connect(lambda: self._on_move_rel(-abs(float(self.sp_rel.value()))))
        self.btn_rel_pos.clicked.connect(lambda: self._on_move_rel(abs(float(self.sp_rel.value()))))
        self.btn_bkpos.clicked.connect(self._on_bkpos_clicked)
        self.btn_zero.clicked.connect(self._on_zero_clicked)
        self.btn_home.clicked.connect(self._on_home_clicked)
        self.chk_scurve.toggled.connect(self._on_scurve_toggled)

    def _on_scurve_toggled(self, checked):
        mode_val = 1 if checked else 0
        self.main_window.send_compact(f"CMD|set_axis_mode|{self.axis}|{mode_val}")

    def set_moving_state(self, is_moving: bool):
        if self.property("moving") != is_moving:
            self.setProperty("moving", is_moving)
            self.style().unpolish(self)
            self.style().polish(self)
            self.update()

    def set_homed_state(self, is_homed: bool):
        self.is_homed = is_homed
        if self.property("homed") != is_homed:
            self.setProperty("homed", is_homed)
            self.style().unpolish(self)
            self.style().polish(self)
            self.update()

    def update_data(self, d: dict):
        if not d: return
        pos = float(d.get("pos", 0.0))
        
        # Filtro estético para limpiar micro-residuos numéricos de punto flotante
        if abs(pos) < 0.001:
            pos = 0.000

        max_travel = float(d.get("max_travel", 0.0))
        homed = bool(d.get("homed", False))
        is_moving = bool(d.get("is_moving", False))
        direction = str(d.get("move_dir", "none")).upper()
        err = str(d.get("last_error", "") or "").strip()

        if is_moving and not self.was_moving:
            self.last_start_pos = pos
            self.lbl_lpos_val.setText(f"{self.last_start_pos:.3f}")
        self.was_moving = is_moving

        self.lbl_pos_num.setText(f"{pos:.3f} mm")
        self.lbl_woff.setText(f"{max_travel:.3f}")
        self.lbl_dir.setText(direction if is_moving else "STOP")
        self.lbl_dir.setStyleSheet("color: #58a6ff; font-weight: bold; font-size: 14px;" if is_moving else "color: #8b949e; font-weight: bold; font-size: 14px;")

        self.set_moving_state(is_moving)
        self.set_homed_state(homed)

        if homed:
            self.lbl_homing.setText("HOMED")
            self.lbl_homing.setStyleSheet("background-color: #1a472a; color: #3fb950; font-weight: bold; border-radius: 3px; padding: 2px; font-size: 12px;")
        else:
            self.lbl_homing.setText("NO HOME")
            self.lbl_homing.setStyleSheet("background-color: #551a1a; color: #ff7b72; font-weight: bold; border-radius: 3px; padding: 2px; font-size: 12px;")

        self.lbl_error.setText(err if err else "Sin error")

    def _on_move_abs(self):
        val = float(self.sp_abs.value())
        if self.main_window._check_target_inside_soft_limit_for_axis(self.axis, val):
            cur_pos = float(self.main_window.last_status.get("axes", {}).get(self.axis, {}).get("pos", 0.0))
            self.last_start_pos = cur_pos
            self.lbl_lpos_val.setText(f"{self.last_start_pos:.3f}")
            self.main_window.send_compact(f"CMD|move_axis_abs|{self.axis}|{val:.3f}")

    def _on_move_rel(self, delta):
        cur_pos = float(self.main_window.last_status.get("axes", {}).get(self.axis, {}).get("pos", 0.0))
        target = cur_pos + delta
        if self.main_window._check_target_inside_soft_limit_for_axis(self.axis, target):
            self.last_start_pos = cur_pos
            self.lbl_lpos_val.setText(f"{self.last_start_pos:.3f}")
            self.main_window.send_compact(f"CMD|move_axis_rel|{self.axis}|{float(delta):.3f}")

    def _on_bkpos_clicked(self):
        target = self.last_start_pos
        if self.main_window._check_target_inside_soft_limit_for_axis(self.axis, target):
            cur_pos = float(self.main_window.last_status.get("axes", {}).get(self.axis, {}).get("pos", 0.0))
            self.last_start_pos = cur_pos
            self.lbl_lpos_val.setText(f"{self.last_start_pos:.3f}")
            self.main_window.send_compact(f"CMD|move_axis_abs|{self.axis}|{target:.3f}")

    def _on_zero_clicked(self):
        self.main_window.send_compact(f"CMD|set_zero_axis|{self.axis}")

    def _on_home_clicked(self):
        self.main_window.send_single_home(self.axis)


class AxisTab(QWidget):
    def __init__(self, axis_name: str, main_window):
        super().__init__()
        self.axis = axis_name.lower()
        self.main_window = main_window
        self.current_spm = 568.0
        self.raw_step_count = 0
        self.calib_step = 0
        self.dist1 = 0.0; self.steps1 = 0; self.spm1 = 0.0
        self.dist_total = 0.0; self.steps_total = 0; self.spm2 = 0.0
        self.initial_manual_us = 1000
        self.calib_dialog = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(3)

        row_top_btns = QHBoxLayout()
        row_top_btns.setSpacing(3)
        self.btn_open_calibration = QPushButton(f"Calibración 2P ({self.axis.upper()})")
        self.btn_open_calibration.setEnabled(False)
        self.btn_invert_dir = QPushButton("Invertir Sentido Eje")
        self.btn_invert_dir.setStyleSheet("background-color: #8957e5; color: white;")
        self.btn_invert_dir.setToolTip("Ejecutar 1 sola vez si el eje se mueve hacia el sensor al pulsar (+)")
        row_top_btns.addWidget(self.btn_open_calibration, 1)
        row_top_btns.addWidget(self.btn_invert_dir, 1)
        lay.addLayout(row_top_btns)

        self.lbl_pasos_acumulados = QLabel("---")
        self.lbl_calculated_spm = QLabel("---")
        self.btn_calculate_preview = QPushButton("Calcular")
        self.btn_calculate_preview.setEnabled(False)
        self.sp_distancia = QDoubleSpinBox()
        self.sp_distancia.setRange(0.0, 5000.0); self.sp_distancia.setDecimals(3); self.sp_distancia.setEnabled(False)
        self.lbl_calib_instructions = QLabel("Listo para calibrar.")
        self.lbl_last_calib_date = QLabel("---")
        self.btn_start_wizard = QPushButton("Iniciar P1 (100% Vel)")
        self.btn_confirm_pass = QPushButton("Confirmar Pasada"); self.btn_confirm_pass.setEnabled(False)

        gb_prof = QGroupBox("Perfil Curva en S (Jerk Suave)")
        form_prof = QFormLayout(gb_prof)
        form_prof.setContentsMargins(6, 6, 6, 6)
        form_prof.setSpacing(3)

        self.sp_start_mms = QDoubleSpinBox(); self.sp_start_mms.setRange(0.1, 200.0); self.sp_start_mms.setValue(1.5); self.sp_start_mms.setFixedWidth(80)
        self.sp_cruise_mms = QDoubleSpinBox(); self.sp_cruise_mms.setRange(0.5, 300.0); self.sp_cruise_mms.setValue(15.0); self.sp_cruise_mms.setFixedWidth(80)
        self.sp_end_mms = QDoubleSpinBox(); self.sp_end_mms.setRange(0.1, 200.0); self.sp_end_mms.setValue(1.5); self.sp_end_mms.setFixedWidth(80)
        self.sp_ramp_ratio = QDoubleSpinBox(); self.sp_ramp_ratio.setRange(0.05, 0.45); self.sp_ramp_ratio.setValue(0.25); self.sp_ramp_ratio.setSingleStep(0.05); self.sp_ramp_ratio.setFixedWidth(80)

        row_speeds1 = QHBoxLayout(); row_speeds1.setSpacing(4)
        row_speeds1.addWidget(self.sp_start_mms)
        row_speeds1.addWidget(QLabel("Cru:"))
        row_speeds1.addWidget(self.sp_cruise_mms)
        row_speeds1.addStretch(1)

        row_speeds2 = QHBoxLayout(); row_speeds2.setSpacing(4)
        row_speeds2.addWidget(self.sp_end_mms)
        row_speeds2.addWidget(QLabel("Rampa:"))
        row_speeds2.addWidget(self.sp_ramp_ratio)
        row_speeds2.addStretch(1)

        self.sp_manual_us = QSpinBox(); self.sp_manual_us.setRange(10, 20000); self.sp_manual_us.setValue(1000); self.sp_manual_us.setFixedWidth(80)
        self.sp_jog_us = QSpinBox(); self.sp_jog_us.setRange(10, 20000); self.sp_jog_us.setValue(1000); self.sp_jog_us.setFixedWidth(80)

        row_control_us = QHBoxLayout(); row_control_us.setSpacing(4)
        row_control_us.addWidget(self.sp_manual_us)
        row_control_us.addWidget(QLabel("Jog:"))
        row_control_us.addWidget(self.sp_jog_us)
        row_control_us.addStretch(1)

        self.btn_set_profile = QPushButton("Guardar Perfil S-Curve")

        form_prof.addRow("Inicio/Cru (mm/s):", row_speeds1)
        form_prof.addRow("Fin/Rampa S:", row_speeds2)
        form_prof.addRow("Manual/Jog (us):", row_control_us)
        form_prof.addRow(self.btn_set_profile)
        lay.addWidget(gb_prof)

        gb_lim = QGroupBox("Límites y Homing")
        form_lim = QFormLayout(gb_lim)
        form_lim.setContentsMargins(6, 6, 6, 6)
        form_lim.setSpacing(3)

        self.sp_max_travel = QDoubleSpinBox(); self.sp_max_travel.setRange(1.0, 5000.0); self.sp_max_travel.setValue(110.0); self.sp_max_travel.setFixedWidth(85)
        self.sp_backoff_steps = QSpinBox(); self.sp_backoff_steps.setRange(1, 100000); self.sp_backoff_steps.setValue(1136); self.sp_backoff_steps.setFixedWidth(85)
        self.sp_soft_offset_steps = QSpinBox(); self.sp_soft_offset_steps.setRange(1, 200000); self.sp_soft_offset_steps.setValue(2840); self.sp_soft_offset_steps.setFixedWidth(85)

        form_lim.addRow("Max (mm):", self.sp_max_travel)
        form_lim.addRow("Backoff (stp):", self.sp_backoff_steps)
        form_lim.addRow("Soft Off (stp):", self.sp_soft_offset_steps)

        self.sp_hseek = QSpinBox(); self.sp_hseek.setRange(10, 20000); self.sp_hseek.setValue(1200); self.sp_hseek.setFixedWidth(80)
        self.sp_hfeed = QSpinBox(); self.sp_hfeed.setRange(10, 20000); self.sp_hfeed.setValue(2800); self.sp_hfeed.setFixedWidth(80)
        self.sp_hbo = QSpinBox(); self.sp_hbo.setRange(10, 20000); self.sp_hbo.setValue(1500); self.sp_hbo.setFixedWidth(85)

        row_hspeeds = QHBoxLayout(); row_hspeeds.setSpacing(4)
        row_hspeeds.addWidget(self.sp_hseek)
        row_hspeeds.addWidget(QLabel("Feed:"))
        row_hspeeds.addWidget(self.sp_hfeed)
        row_hspeeds.addStretch(1)

        form_lim.addRow("Seek/Feed (us):", row_hspeeds)
        form_lim.addRow("Homing BO (us):", self.sp_hbo)

        self.btn_set_limits = QPushButton("Guardar Límites")
        form_lim.addRow(self.btn_set_limits)
        lay.addWidget(gb_lim)

        lay.addStretch(1)

        self.btn_open_calibration.clicked.connect(self._open_calibration_dialog)
        self.btn_invert_dir.clicked.connect(self._on_invert_dir_clicked)
        self.btn_set_limits.clicked.connect(lambda: self.main_window.send_homing_vars())

    def _on_invert_dir_clicked(self):
        resp = QMessageBox.question(
            self, "Sentido de Eje",
            f"¿Desea invertir el sentido de giro eléctrico del eje {self.axis.upper()}?\n"
            "Esta acción queda grabada de forma permanente en la memoria interna del ESP32.",
            QMessageBox.Yes | QMessageBox.No
        )
        if resp == QMessageBox.Yes:
            self.main_window.send_compact(f"CMD|invert_axis_dir|{self.axis}")

    def _open_calibration_dialog(self):
        if not self.calib_dialog or not self.calib_dialog.isVisible():
            self.calib_dialog = CalibrationDialog(self, self.main_window)
            self.calib_dialog.show()
        else:
            self.calib_dialog.raise_()
            self.calib_dialog.activateWindow()

    def set_steps_count(self, steps: int):
        self.raw_step_count = steps
        if self.calib_step > 0: self.lbl_pasos_acumulados.setText(f"{steps} stp")
        else: self.lbl_pasos_acumulados.setText("---")
        if self.calib_dialog and self.calib_dialog.isVisible():
            self.calib_dialog.sync_ui()

    def _calculate_preview(self):
        steps = float(abs(self.raw_step_count))
        mm = float(self.sp_distancia.value())
        if steps > 0 and mm > 0.0:
            spm = steps / mm
            self.lbl_calculated_spm.setText(f"{spm:.4f} stp/mm")
        else:
            QMessageBox.warning(self, "Cálculo", "Pasos > 0 y distancia > 0 requeridos.")

    def _start_calibration_wizard(self):
        ax = self.axis
        act_ok = bool(self.main_window.last_status.get("actuators_enabled", False))
        if not act_ok:
            QMessageBox.warning(self, "Actuadores", "Active los actuadores antes de calibrar.")
            return

        self.calib_step = 1
        self.initial_manual_us = int(self.sp_manual_us.value())
        self.main_window.send_compact(f"CMD|reset_step_counter|{ax}")
        self.sp_distancia.setEnabled(True); self.sp_distancia.setValue(0.0)
        self.btn_confirm_pass.setEnabled(True); self.btn_confirm_pass.setText("Confirmar P1")
        self.btn_calculate_preview.setEnabled(True); self.btn_start_wizard.setEnabled(False)
        self.lbl_pasos_acumulados.setText("0 stp")
        self.lbl_calib_instructions.setText("P1 (100%): Mueva eje (+). Mida distancia y confirme.")

    def _confirm_pass_measurement(self):
        val = float(self.sp_distancia.value())
        if val <= 0.0:
            QMessageBox.warning(self, "Valor Inválido", "Distancia debe ser mayor a 0.")
            return

        if self.calib_step == 1:
            self.dist1 = val
            self.steps1 = abs(self.raw_step_count)
            if self.steps1 <= 0:
                QMessageBox.warning(self, "Error", "No se detectaron pasos.")
                return
            self.spm1 = float(self.steps1) / self.dist1
            self.calib_step = 2

            speed_75_pct = int(self.initial_manual_us / 0.75)
            self.main_window.send_compact(f"CMD|set_manual_speed_axis|{self.axis}|{speed_75_pct}|{int(self.sp_jog_us.value())}")
            self.sp_distancia.setValue(0.0); self.btn_confirm_pass.setText("Confirmar P2")
            self.lbl_calib_instructions.setText(f"P1: {self.spm1:.2f} stp/mm.\nP2 (75%): Mueva eje (+) e ingrese distancia acumulada.")

        elif self.calib_step == 2:
            self.dist_total = val; self.steps_total = abs(self.raw_step_count)
            if self.steps_total <= 0:
                QMessageBox.warning(self, "Error", "Sin pasos en P2.")
                return

            self.spm2 = float(self.steps_total) / self.dist_total
            self.spm_final = (self.spm1 + self.spm2) / 2.0
            self.main_window.send_compact(f"CMD|set_manual_speed_axis|{self.axis}|{self.initial_manual_us}|{int(self.sp_jog_us.value())}")

            fecha_calib = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.main_window.send_compact(f"CMD|set_calibration_axis|{self.axis}|{self.spm_final:.4f}|{self.dist_total:.2f}|{fecha_calib}")

            self.current_spm = self.spm_final
            self.lbl_calculated_spm.setText(f"{self.spm_final:.4f} stp/mm")
            self.lbl_last_calib_date.setText(fecha_calib)
            self.sp_max_travel.setValue(self.dist_total)

            QMessageBox.information(self, "Guardado", f"SPM Promedio: {self.spm_final:.4f} stp/mm.")
            self.calib_step = 0
            self.btn_start_wizard.setEnabled(True)
            self.btn_confirm_pass.setEnabled(False)
            self.btn_calculate_preview.setEnabled(False)
            self.sp_distancia.setEnabled(False)
            self.lbl_calib_instructions.setText("Calibración completada.")


class SequenceProgrammerWidget(QWidget):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self.main_window = main_window

        self.sequence_queue = []
        self.current_step = 0
        self.is_running = False

        self.exec_timer = QTimer(self)
        self.exec_timer.timeout.connect(self._step_sequence)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(3)

        gb_in = QGroupBox("Editor de Orden")
        grid_in = QGridLayout(gb_in)
        grid_in.setContentsMargins(4, 4, 4, 4)
        grid_in.setSpacing(3)

        self.cb_axis = QComboBox()
        self.cb_axis.addItems(["X", "Y", "Z", "W"])
        self.cb_axis.setFixedWidth(50)

        self.cb_action = QComboBox()
        self.cb_action.addItems(["Move Abs", "Move Rel (+)", "Move Rel (-)", "Set Zero", "Pausa (s)"])

        self.sp_value = QDoubleSpinBox()
        self.sp_value.setRange(-5000.0, 5000.0)
        self.sp_value.setDecimals(3)
        self.sp_value.setValue(10.0)
        self.sp_value.setFixedWidth(75)

        self.btn_add_cmd = QPushButton("Insertar Orden")
        self.btn_add_cmd.setStyleSheet("background-color: #238636; color: white;")

        grid_in.addWidget(QLabel("Eje:"), 0, 0)
        grid_in.addWidget(self.cb_axis, 0, 1)
        grid_in.addWidget(QLabel("Tipo:"), 0, 2)
        grid_in.addWidget(self.cb_action, 0, 3)
        grid_in.addWidget(QLabel("Valor:"), 0, 4)
        grid_in.addWidget(self.sp_value, 0, 5)
        grid_in.addWidget(self.btn_add_cmd, 0, 6)
        lay.addWidget(gb_in)

        self.list_orders = QListWidget()
        self.list_orders.setStyleSheet("background-color: #0d1117; font-family: Consolas; font-size: 11px; color: #00f0ff;")
        lay.addWidget(self.list_orders, 1)

        row_list_ops = QHBoxLayout()
        row_list_ops.setSpacing(3)

        self.btn_del = QPushButton("Eliminar")
        self.btn_clear = QPushButton("Limpiar")
        self.btn_up = QPushButton("▲ Subir")
        self.btn_down = QPushButton("▼ Bajar")
        self.btn_open = QPushButton("Abrir...")
        self.btn_save = QPushButton("Guardar...")

        row_list_ops.addWidget(self.btn_del)
        row_list_ops.addWidget(self.btn_clear)
        row_list_ops.addWidget(self.btn_up)
        row_list_ops.addWidget(self.btn_down)
        row_list_ops.addStretch(1)
        row_list_ops.addWidget(self.btn_open)
        row_list_ops.addWidget(self.btn_save)
        lay.addLayout(row_list_ops)

        row_exec = QHBoxLayout()
        row_exec.setSpacing(4)

        self.lbl_seq_status = QLabel("Listo.")
        self.lbl_seq_status.setStyleSheet("font-weight: bold; color: #7ee787; font-size: 12px;")

        self.btn_run = QPushButton("EJECUTAR PROGRAMA")
        self.btn_run.setStyleSheet("background-color: #1f6feb; color: white; font-weight: bold; font-size: 12px; padding: 5px;")

        self.btn_stop = QPushButton("DETENER")
        self.btn_stop.setStyleSheet("background-color: #a42424; color: white; font-weight: bold; font-size: 12px; padding: 5px;")
        self.btn_stop.setEnabled(False)

        row_exec.addWidget(self.lbl_seq_status, 1)
        row_exec.addWidget(self.btn_run)
        row_exec.addWidget(self.btn_stop)
        lay.addLayout(row_exec)

        self.btn_add_cmd.clicked.connect(self._add_order)
        self.btn_del.clicked.connect(self._delete_order)
        self.btn_clear.clicked.connect(self.list_orders.clear)
        self.btn_up.clicked.connect(self._move_order_up)
        self.btn_down.clicked.connect(self._move_order_down)
        self.btn_save.clicked.connect(self._save_file)
        self.btn_open.clicked.connect(self._open_file)
        self.btn_run.clicked.connect(self._start_sequence)
        self.btn_stop.clicked.connect(self._stop_sequence)

    def update_available_axes(self, axis_blocks: dict):
        current_axis = self.cb_axis.currentText()
        self.cb_axis.blockSignals(True)
        self.cb_axis.clear()

        for a in ["x", "y", "z", "w"]:
            blk = axis_blocks.get(a)
            if blk and blk.is_homed:
                self.cb_axis.addItem(a.upper())

        idx = self.cb_axis.findText(current_axis)
        if idx >= 0:
            self.cb_axis.setCurrentIndex(idx)
        self.cb_axis.blockSignals(False)

        has_homed_axis = self.cb_axis.count() > 0
        self.setEnabled(has_homed_axis)

    def _add_order(self):
        ax = self.cb_axis.currentText()
        if not ax:
            QMessageBox.warning(self, "Secuenciador", "No hay ningún eje referenciado (Homed) disponible.")
            return

        act = self.cb_action.currentText()
        val = self.sp_value.value()
        order_str = f"[{ax}] {act}: {val:.3f}"
        self.list_orders.addItem(order_str)

    def _delete_order(self):
        for item in self.list_orders.selectedItems():
            self.list_orders.takeItem(self.list_orders.row(item))

    def _move_order_up(self):
        row = self.list_orders.currentRow()
        if row > 0:
            item = self.list_orders.takeItem(row)
            self.list_orders.insertItem(row - 1, item)
            self.list_orders.setCurrentRow(row - 1)

    def _move_order_down(self):
        row = self.list_orders.currentRow()
        if row >= 0 and row < self.list_orders.count() - 1:
            item = self.list_orders.takeItem(row)
            self.list_orders.insertItem(row + 1, item)
            self.list_orders.setCurrentRow(row + 1)

    def _save_file(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Guardar Secuencia", "", "Secuencia CNC (*.json *.txt)")
        if not fn: return
        data = [self.list_orders.item(i).text() for i in range(self.list_orders.count())]
        try:
            with open(fn, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            QMessageBox.information(self, "Guardado", "Secuencia guardada con éxito.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo guardar: {e}")

    def _open_file(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Abrir Secuencia", "", "Secuencia CNC (*.json *.txt)")
        if not fn: return
        try:
            with open(fn, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.list_orders.clear()
            for line in data:
                self.list_orders.addItem(str(line))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo abrir: {e}")

    def _start_sequence(self):
        if self.list_orders.count() == 0:
            QMessageBox.warning(self, "Secuenciador", "La lista de órdenes está vacía.")
            return

        self.sequence_queue = [self.list_orders.item(i).text() for i in range(self.list_orders.count())]
        self.current_step = 0
        self.is_running = True
        self.btn_run.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lbl_seq_status.setText(f"Ejecutando paso 1/{len(self.sequence_queue)}...")
        self.exec_timer.start(50)

    def _stop_sequence(self):
        self.is_running = False
        self.exec_timer.stop()
        self.btn_run.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_seq_status.setText("Secuencia detenida.")
        self.main_window.on_stop_clicked()

    def _step_sequence(self):
        if not self.is_running:
            return

        is_moving = False
        axes_data = self.main_window.last_status.get("axes", {})
        for a_info in axes_data.values():
            if a_info.get("is_moving", False):
                is_moving = True
                break

        if is_moving:
            return

        if self.current_step >= len(self.sequence_queue):
            self.is_running = False
            self.exec_timer.stop()
            self.btn_run.setEnabled(True)
            self.btn_stop.setEnabled(False)
            self.lbl_seq_status.setText("Secuencia completada con éxito.")
            QMessageBox.information(self, "Secuenciador", "Secuencia finalizada.")
            return

        cmd_txt = self.sequence_queue[self.current_step]
        self.list_orders.setCurrentRow(self.current_step)
        self.lbl_seq_status.setText(f"Paso {self.current_step + 1}/{len(self.sequence_queue)}: {cmd_txt}")

        try:
            bracket_close = cmd_txt.find("]")
            ax_name = cmd_txt[1:bracket_close].lower()
            rest = cmd_txt[bracket_close + 1:].strip()
            action, val_s = rest.split(":")
            action = action.strip()
            val = float(val_s.strip())

            if not self.main_window.axis_blocks[ax_name].is_homed:
                self._stop_sequence()
                QMessageBox.critical(self, "Error de Secuencia", f"Eje {ax_name.upper()} no está referenciado.")
                return

            if action == "Move Abs":
                if self.main_window._check_target_inside_soft_limit_for_axis(ax_name, val):
                    self.main_window.send_compact(f"CMD|move_axis_abs|{ax_name}|{val:.3f}")
            elif action == "Move Rel (+)":
                cur_pos = float(self.main_window.last_status.get("axes", {}).get(ax_name, {}).get("pos", 0.0))
                if self.main_window._check_target_inside_soft_limit_for_axis(ax_name, cur_pos + abs(val)):
                    self.main_window.send_compact(f"CMD|move_axis_rel|{ax_name}|{abs(val):.3f}")
            elif action == "Move Rel (-)":
                cur_pos = float(self.main_window.last_status.get("axes", {}).get(ax_name, {}).get("pos", 0.0))
                if self.main_window._check_target_inside_soft_limit_for_axis(ax_name, cur_pos - abs(val)):
                    self.main_window.send_compact(f"CMD|move_axis_rel|{ax_name}|{-abs(val):.3f}")
            elif action == "Set Zero":
                self.main_window.send_compact(f"CMD|set_zero_axis|{ax_name}")
            elif action == "Pausa (s)":
                self.exec_timer.setInterval(int(max(10, val * 1000)))
                self.current_step += 1
                return

            self.exec_timer.setInterval(100)
            self.current_step += 1

        except Exception as e:
            self._stop_sequence()
            QMessageBox.critical(self, "Error", f"Error en orden: {e}")


class MainWindow(QMainWindow):
    AXES = ["x", "y", "z", "w"]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Controlador CNC XYZW - NET-LOG & GRBL")
        self.setFixedSize(1350, 730)

        self.worker = None
        self.last_status = {}
        self.last_machine_state = ""
        self._first_sync_done = False
        self._dpad_active = False
        self._dpad_axis_group = "xy"
        self.use_grbl_protocol = False

        self._homing_queue = []
        self._homing_timer = QTimer(self)
        self._homing_timer.timeout.connect(self._process_homing_queue)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self.poll_status)

        self._deadman_timer = QTimer(self)
        self._deadman_timer.timeout.connect(self._send_deadman_ping)

        self._build_ui()
        self._apply_style()
        self._set_initial_disconnected_ui()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape: self.close()
        super().keyPressEvent(event)

    def _set_initial_disconnected_ui(self):
        if self._poll_timer.isActive():
            self._poll_timer.stop()
        if self._deadman_timer.isActive():
            self._deadman_timer.stop()

        self.lbl_conn_text.setText("DESCONECTADO")
        self._set_label_color_state(self.lbl_conn_text, "red")
        self.lbl_actuators_text.setText("DESHABILITADOS")
        self._set_label_color_state(self.lbl_actuators_text, "red")
        self.lbl_machine_state_v.setText("---")
        self._set_label_color_state(self.lbl_machine_state_v, "default")
        
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.last_status = {}
        for b in self.axis_blocks.values():
            b.set_homed_state(False)
        self._update_controls_interlock()

    def _build_ui(self):
        root = QWidget(); self.setCentralWidget(root)
        main_layout = QHBoxLayout(root)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # Panel Izquierdo (340px)
        left = QWidget(); left.setFixedWidth(340)
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(3)

        gb_conn_left = QGroupBox("Configuración de Red & Protocolo")
        form_conn = QFormLayout(gb_conn_left)
        form_conn.setContentsMargins(6, 6, 6, 6); form_conn.setSpacing(4)
        self.ed_host = QLineEdit("192.168.1.167")
        self.ed_port = QSpinBox(); self.ed_port.setRange(1, 65535); self.ed_port.setValue(5000)
        
        self.btn_connect = QPushButton("Conectar")
        self.btn_disconnect = QPushButton("Desconectar"); self.btn_disconnect.setEnabled(False)
        row_c_btns = QHBoxLayout(); row_c_btns.setSpacing(4)
        row_c_btns.addWidget(self.btn_connect); row_c_btns.addWidget(self.btn_disconnect)

        row_proto = QHBoxLayout()
        row_proto.setSpacing(4)
        self.lbl_proto_left = QLabel("NET-LOG")
        self.lbl_proto_left.setStyleSheet("font-weight: bold; color: #58a6ff;")
        self.switch_proto = ToggleSwitch()
        self.lbl_proto_right = QLabel("GRBL")
        self.lbl_proto_right.setStyleSheet("font-weight: bold; color: #8b949e;")
        row_proto.addWidget(self.lbl_proto_left)
        row_proto.addWidget(self.switch_proto)
        row_proto.addWidget(self.lbl_proto_right)
        row_proto.addStretch(1)

        form_conn.addRow("Host/IP:", self.ed_host)
        form_conn.addRow("Puerto:", self.ed_port)
        form_conn.addRow(row_c_btns)
        form_conn.addRow("Protocolo:", row_proto)
        left_layout.addWidget(gb_conn_left)

        gb_tabs = QGroupBox("Parámetros & Calibración de Ejes")
        vtabs = QVBoxLayout(gb_tabs)
        vtabs.setContentsMargins(4, 4, 4, 4)
        self.tabs = QTabWidget()
        self.axis_tabs = {}
        for a in self.AXES:
            t = AxisTab(a, self)
            self.axis_tabs[a] = t
            self.tabs.addTab(t, a.upper())
        vtabs.addWidget(self.tabs)
        left_layout.addWidget(gb_tabs)

        gb_empty = QGroupBox("Espacio Auxiliar / Diagnóstico")
        empty_layout = QVBoxLayout(gb_empty)
        empty_lbl = QLabel("[ Sin datos auxiliares ]")
        empty_lbl.setAlignment(Qt.AlignCenter)
        empty_lbl.setStyleSheet("color: #484f58; font-style: italic;")
        empty_layout.addWidget(empty_lbl)
        left_layout.addWidget(gb_empty, 1)

        # Panel Central (460px)
        center = QWidget(); center.setFixedWidth(460)
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(3)

        gb_sys = QGroupBox("Estado del Sistema")
        sys_grid = QGridLayout(gb_sys)
        sys_grid.setContentsMargins(4, 4, 4, 4)
        sys_grid.setHorizontalSpacing(4); sys_grid.setVerticalSpacing(2)

        self.lbl_conn_text = QLabel("DESCONECTADO")
        self.lbl_conn_text.setStyleSheet("font-size: 13px; font-weight: bold;")
        self.lbl_actuators_text = QLabel("DESHABILITADOS")
        self.lbl_actuators_text.setStyleSheet("font-size: 13px; font-weight: bold;")
        self.btn_enable_actuators = QPushButton("Activar Actuadores"); self.btn_enable_actuators.setCheckable(True)
        self.btn_all_homing = QPushButton("ALL HOMING")
        self.btn_all_homing.setProperty("homingBtn", True)
        self.lbl_machine_state_v = QLabel("---")
        self.lbl_machine_state_v.setStyleSheet("font-size: 13px; font-weight: bold;")

        sys_grid.addWidget(QLabel("Conexión:"), 0, 0); sys_grid.addWidget(self.lbl_conn_text, 0, 1)
        sys_grid.addWidget(QLabel("Actuadores:"), 1, 0); sys_grid.addWidget(self.lbl_actuators_text, 1, 1); sys_grid.addWidget(self.btn_enable_actuators, 1, 2)
        sys_grid.addWidget(QLabel("Máquina:"), 2, 0); sys_grid.addWidget(self.lbl_machine_state_v, 2, 1); sys_grid.addWidget(self.btn_all_homing, 2, 2)
        center_layout.addWidget(gb_sys)

        self.axis_blocks = {}
        for a in self.AXES:
            blk = AxisBlock(a, self)
            blk.btn_home.setProperty("homingBtn", True)
            self.axis_blocks[a] = blk
            center_layout.addWidget(blk)

        gb_center_empty = QGroupBox("Estado Operativo de Ejes")
        c_empty_layout = QVBoxLayout(gb_center_empty)
        c_empty_lbl = QLabel("[ Sin alarmas pendientes ]")
        c_empty_lbl.setAlignment(Qt.AlignCenter)
        c_empty_lbl.setStyleSheet("color: #484f58; font-style: italic;")
        c_empty_layout.addWidget(c_empty_lbl)
        center_layout.addWidget(gb_center_empty, 1)

        # Panel Derecho Ajustado a 526px para Ventana de 1350px
        right = QWidget(); right.setFixedWidth(526)
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(3)

        self.right_tabs = QTabWidget()

        # Visor Óptico
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

        row1 = QHBoxLayout(); row1.setSpacing(3)
        self.cb_cam_idx = QComboBox(); self.cb_cam_idx.addItems(["CAM 0", "CAM 1", "CAM 2"]); self.cb_cam_idx.setFixedWidth(68)
        self.btn_cam_toggle = QPushButton("Iniciar Cámara"); self.btn_cam_toggle.setFixedWidth(95)

        self.slider_zoom = QSlider(Qt.Horizontal)
        self.slider_zoom.setRange(100, 500); self.slider_zoom.setValue(100); self.slider_zoom.setFixedWidth(75)
        self.lbl_zoom = QLabel("1.0x"); self.lbl_zoom.setFixedWidth(28)

        self.sp_calib_px = QDoubleSpinBox()
        self.sp_calib_px.setRange(1.0, 5000.0); self.sp_calib_px.setDecimals(1); self.sp_calib_px.setValue(100.0)
        self.sp_calib_px.setToolTip("Píxeles para patrón de 10.0 mm"); self.sp_calib_px.setFixedWidth(58)
        self.btn_set_calib = QPushButton("Fijar Escala"); self.btn_set_calib.setFixedWidth(70)

        row1.addWidget(self.cb_cam_idx)
        row1.addWidget(self.btn_cam_toggle)
        row1.addWidget(QLabel("Zoom:"))
        row1.addWidget(self.slider_zoom)
        row1.addWidget(self.lbl_zoom)
        row1.addWidget(QLabel("Px/10mm:"))
        row1.addWidget(self.sp_calib_px)
        row1.addWidget(self.btn_set_calib)

        row2 = QHBoxLayout(); row2.setSpacing(3)
        self.chk_grid = QCheckBox("Grid")
        self.chk_grid.setChecked(True)

        self.slider_grid = QSlider(Qt.Horizontal)
        self.slider_grid.setRange(10, 150); self.slider_grid.setValue(40); self.slider_grid.setFixedWidth(80)
        self.lbl_grid_val = QLabel("40px"); self.lbl_grid_val.setFixedWidth(32)

        self.btn_clear_meas = QPushButton("Borrar Medición")
        self.btn_clear_meas.setStyleSheet("background-color: #3b202a; border-color: #6e273b; color: #ff7b72;")

        row2.addWidget(self.chk_grid)
        row2.addWidget(QLabel("Paso:"))
        row2.addWidget(self.slider_grid)
        row2.addWidget(self.lbl_grid_val)
        row2.addStretch(1)
        row2.addWidget(self.btn_clear_meas)

        cam_tool_layout.addLayout(row1)
        cam_tool_layout.addLayout(row2)
        cam_layout.addWidget(cam_toolbar)

        # Secuenciador
        self.sequence_widget = SequenceProgrammerWidget(self)

        self.right_tabs.addTab(tab_cam_container, "Visor Óptico USB")
        self.right_tabs.addTab(self.sequence_widget, "Secuenciador CNC")

        # Control Manual D-Pad
        gb_dpad = QGroupBox("Control Manual D-Pad XY / Control ZW")
        dpad_container_layout = QVBoxLayout(gb_dpad)
        dpad_container_layout.setContentsMargins(0, 2, 0, 2)

        dpad_controls_row = QHBoxLayout()
        dpad_controls_row.setContentsMargins(0, 0, 0, 0)
        dpad_controls_row.setSpacing(0)

        labels_xy = {"up": "Y+", "down": "Y-", "left": "X-", "right": "X+"}
        self.dpad_xy = DPadPainter(labels=labels_xy)
        self.dpad_zw = VerticalZWControl()

        dpad_controls_row.addStretch(1)
        dpad_controls_row.addWidget(self.dpad_xy)
        dpad_controls_row.addStretch(1)
        dpad_controls_row.addWidget(self.dpad_zw)
        dpad_controls_row.addStretch(1)

        dpad_container_layout.addStretch(1)
        dpad_container_layout.addLayout(dpad_controls_row)
        dpad_container_layout.addStretch(1)

        right_layout.addWidget(self.right_tabs, 58)
        right_layout.addWidget(gb_dpad, 42)

        main_layout.addWidget(left)
        main_layout.addWidget(center)
        main_layout.addWidget(right)

        # Conexiones
        self.btn_cam_toggle.clicked.connect(self._toggle_camera)
        self.chk_grid.toggled.connect(self.cam_widget.set_grid_enabled)
        self.slider_grid.valueChanged.connect(self._on_grid_slider_changed)
        self.slider_zoom.valueChanged.connect(self._on_zoom_slider_changed)
        self.btn_set_calib.clicked.connect(self._on_set_calibration_clicked)
        self.btn_clear_meas.clicked.connect(self.cam_widget.clear_measurements)

        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)
        self.btn_enable_actuators.clicked.connect(self.send_enable_actuators)
        self.btn_all_homing.clicked.connect(self.start_all_homing)
        self.switch_proto.toggled.connect(self._on_proto_switch_toggled)
        self.tabs.currentChanged.connect(self._on_tab_changed)

        for ax_name, tab in self.axis_tabs.items():
            tab.btn_set_profile.clicked.connect(lambda _, a=ax_name, t=tab: self.on_set_profile_and_speed(a, t))

        self.dpad_xy.directionPressed.connect(lambda d: self._dpad_press_group("xy", d))
        self.dpad_xy.directionReleased.connect(lambda d: self._dpad_release_direction("xy", d))
        self.dpad_xy.stopPressed.connect(self.on_stop_clicked)
        self.dpad_zw.directionPressed.connect(self._zw_vertical_press)
        self.dpad_zw.directionReleased.connect(self._zw_vertical_release)

        self._on_tab_changed(0)

    def _on_proto_switch_toggled(self, checked: bool):
        self.use_grbl_protocol = checked
        if checked:
            self.lbl_proto_left.setStyleSheet("font-weight: bold; color: #8b949e;")
            self.lbl_proto_right.setStyleSheet("font-weight: bold; color: #7ee787;")
        else:
            self.lbl_proto_left.setStyleSheet("font-weight: bold; color: #58a6ff;")
            self.lbl_proto_right.setStyleSheet("font-weight: bold; color: #8b949e;")

    def _send_deadman_ping(self):
        if self._dpad_active:
            if not self.use_grbl_protocol:
                self.send_compact("CMD|manual_ping")
            else:
                self.send_compact("?")

    def _toggle_camera(self):
        if self.cam_widget.cap and self.cam_widget.cap.isOpened():
            self.cam_widget.stop_camera()
            self.btn_cam_toggle.setText("Iniciar Cámara")
            self.btn_cam_toggle.setStyleSheet("")
        else:
            idx = self.cb_cam_idx.currentIndex()
            ok = self.cam_widget.start_camera(idx)
            if ok:
                self.btn_cam_toggle.setText("Detener Cámara")
                self.btn_cam_toggle.setStyleSheet("background-color: #8c2424; color: white;")
            else:
                QMessageBox.critical(self, "Cámara", f"No se pudo conectar a cámara {idx}.")

    def _on_grid_slider_changed(self, val):
        self.lbl_grid_val.setText(f"{val}px")
        self.cam_widget.set_grid_size(val)

    def _on_zoom_slider_changed(self, val):
        factor = float(val) / 100.0
        self.lbl_zoom.setText(f"{factor:.1f}x")
        self.cam_widget.set_zoom(val)

    def _on_set_calibration_clicked(self):
        val = float(self.sp_calib_px.value())
        self.cam_widget.set_px_per_mm(val)
        QMessageBox.information(self, "Calibración Óptica", f"Escala fijada: {val:.1f} px = 10.0 mm ({val/10.0:.2f} px/mm)")

    def _apply_style(self):
        accent = "#006c6c"
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background-color: #161b22; color: #dbe2ea; font-family: "Segoe UI"; font-size: 11px; }}
            QGroupBox {{ border: 1px solid #2d333b; border-radius: 6px; margin-top: 5px; font-weight: bold; color: {accent}; padding-top: 5px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 6px; padding: 0 4px; }}
            
            QGroupBox[homed="true"] {{ background-color: #11221b; }}
            QGroupBox[homed="false"] {{ background-color: #161b22; }}
            QGroupBox[moving="true"] {{ border: 2px solid #58a6ff; }}
            QGroupBox[moving="false"] {{ border: 1px solid #2d333b; }}
            
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTabWidget::pane {{ background-color: #0d1117; border: 1px solid #30363d; border-radius: 4px; padding: 2px; color: #e6edf3; }}
            QTabBar::tab {{ background: #0d1117; border: 1px solid #30363d; padding: 4px 8px; margin-right: 2px; font-size: 11px; }}
            QTabBar::tab:selected {{ background: #12303b; color: #7ee787; }}
            
            QPushButton {{ background-color: {accent}; border: 1px solid {accent}; border-radius: 4px; padding: 3px 5px; font-weight: bold; color: white; }}
            QPushButton:hover {{ background-color: #008080; border-color: #008080; }}
            QPushButton:disabled {{ background-color: #21262d !important; border: 1px solid #30363d !important; color: #484f58 !important; }}
            
            QPushButton[homingBtn="true"]:enabled {{ background-color: #238636; border-color: #2ea043; color: white; }}
            QPushButton[homingBtn="true"]:hover {{ background-color: #2ea043; }}
            
            QLabel[stateColor="green"] {{ color: #3fb950 !important; font-weight: bold; }}
            QLabel[stateColor="red"] {{ color: #ff4d4f !important; font-weight: bold; }}
            QLabel[stateColor="yellow"] {{ color: #f2cc60 !important; font-weight: bold; }}
            QLabel[stateColor="blue"] {{ color: #58a6ff !important; font-weight: bold; }}
            QLabel[stateColor="default"] {{ color: #dbe2ea !important; font-weight: bold; }}
        """)

    def _set_label_color_state(self, label: QLabel, state: str):
        label.setProperty("stateColor", state)
        label.style().unpolish(label)
        label.style().polish(label)
        label.update()

    def _update_controls_interlock(self):
        is_tcp_connected = bool(self.worker and self.worker.running)
        actuators_ok = bool(self.last_status.get("actuators_enabled", False))
        
        self.btn_connect.setEnabled(not is_tcp_connected)
        self.btn_disconnect.setEnabled(is_tcp_connected)
        self.btn_enable_actuators.setEnabled(is_tcp_connected)

        left_enabled = is_tcp_connected and actuators_ok
        for t in self.axis_tabs.values():
            t.btn_open_calibration.setEnabled(left_enabled)
            t.btn_invert_dir.setEnabled(left_enabled)
            t.btn_set_profile.setEnabled(left_enabled)
            t.btn_set_limits.setEnabled(left_enabled)

        self.btn_all_homing.setEnabled(left_enabled)
        
        for a, b in self.axis_blocks.items():
            b.btn_home.setEnabled(left_enabled)
            axis_homed = b.is_homed and left_enabled
            b.btn_abs.setEnabled(axis_homed)
            b.btn_rel_neg.setEnabled(axis_homed)
            b.btn_rel_pos.setEnabled(axis_homed)
            b.btn_bkpos.setEnabled(axis_homed)
            b.btn_zero.setEnabled(axis_homed)

        xy_homed = left_enabled and (self.axis_blocks["x"].is_homed or self.axis_blocks["y"].is_homed)
        zw_homed = left_enabled and (self.axis_blocks["z"].is_homed or self.axis_blocks["w"].is_homed)

        self.dpad_xy.setEnabled(xy_homed)
        self.dpad_zw.setEnabled(zw_homed)

        self.sequence_widget.update_available_axes(self.axis_blocks)

        self.btn_all_homing.style().unpolish(self.btn_all_homing)
        self.btn_all_homing.style().polish(self.btn_all_homing)
        for b in self.axis_blocks.values():
            b.btn_home.style().unpolish(b.btn_home)
            b.btn_home.style().polish(b.btn_home)

    def _axis_from_tab(self):
        idx = self.tabs.currentIndex()
        if idx < 0: return "x"
        return self.AXES[idx]

    def send_compact(self, line: str):
        if self.worker and self.worker.running:
            self.worker.send_line(line)

    def _dpad_press_group(self, group: str, direction: str):
        if not self.dpad_xy.isEnabled(): return
        if group == "xy":
            if direction == "up": axis, dir_cmd = "y", "forward"
            elif direction == "down": axis, dir_cmd = "y", "backward"
            elif direction == "right": axis, dir_cmd = "x", "forward"
            else: axis, dir_cmd = "x", "backward"

        if not self.axis_blocks[axis].is_homed:
            return

        active_tab = self.axis_tabs[self._axis_from_tab()]
        if active_tab.calib_step > 0 and dir_cmd != "forward":
            self.dpad_xy._pressed = False; self.dpad_xy._active = None; self.dpad_xy.update()
            QMessageBox.warning(self, "Calibración", "Solo avance (+) en calibración.")
            return

        self._dpad_active = True; self._dpad_axis_group = group
        if not self.use_grbl_protocol:
            self.send_compact(f"CMD|manual_start|{axis}|{dir_cmd}")
        else:
            feed = 600
            dist = 1000.0 if dir_cmd == "forward" else -1000.0
            self.send_compact(f"$J=G91 G21 {axis.upper()}{dist:.3f} F{feed}")

        if not self._deadman_timer.isActive():
            self._deadman_timer.start(100)

    def _dpad_release_direction(self, group: str, direction: str):
        self._dpad_active = False
        if self._deadman_timer.isActive():
            self._deadman_timer.stop()

        if group == "xy":
            if not self.use_grbl_protocol:
                if direction in ("up", "down"): self.send_compact("CMD|manual_stop|y")
                elif direction in ("left", "right"): self.send_compact("CMD|manual_stop|x")
            else:
                self.send_compact("\x85")

    def _zw_vertical_press(self, action: str):
        if not self.dpad_zw.isEnabled(): return
        mapping = {"z_up": ("z", "forward"), "z_down": ("z", "backward"), "w_up": ("w", "forward"), "w_down": ("w", "backward")}
        if action in mapping:
            axis, dir_cmd = mapping[action]
            if not self.axis_blocks[axis].is_homed:
                return

            active_tab = self.axis_tabs[self._axis_from_tab()]
            if active_tab.calib_step > 0 and dir_cmd != "forward":
                self.dpad_zw._pressed = False; self.dpad_zw._active = None; self.dpad_zw.update()
                QMessageBox.warning(self, "Calibración", "Solo avance (+) en calibración.")
                return

            self._dpad_active = True; self._dpad_axis_group = "zw"
            if not self.use_grbl_protocol:
                self.send_compact(f"CMD|manual_start|{axis}|{dir_cmd}")
            else:
                feed = 600
                dist = 1000.0 if dir_cmd == "forward" else -1000.0
                self.send_compact(f"$J=G91 G21 {axis.upper()}{dist:.3f} F{feed}")

            if not self._deadman_timer.isActive():
                self._deadman_timer.start(100)

    def _zw_vertical_release(self, action: str):
        self._dpad_active = False
        if self._deadman_timer.isActive():
            self._deadman_timer.stop()

        mapping = {"z_up": "z", "z_down": "z", "w_up": "w", "w_down": "w"}
        if action in mapping:
            if not self.use_grbl_protocol:
                self.send_compact(f"CMD|manual_stop|{mapping[action]}")
            else:
                self.send_compact("\x85")

    def mouseReleaseEvent(self, event):
        if self._dpad_active: self._dpad_release_group(self._dpad_axis_group)
        super().mouseReleaseEvent(event)

    def focusOutEvent(self, event):
        if self._dpad_active: self._dpad_release_group(self._dpad_axis_group)
        super().focusOutEvent(event)

    def _dpad_release_group(self, group: str):
        self._dpad_active = False
        if self._deadman_timer.isActive():
            self._deadman_timer.stop()

        if not self.use_grbl_protocol:
            gaxes = ("x", "y") if group == "xy" else ("z", "w")
            for ax in gaxes: self.send_compact(f"CMD|manual_stop|{ax}")
        else:
            self.send_compact("\x85")

    def on_connect(self):
        host = self.ed_host.text().strip(); port = int(self.ed_port.value())
        self.btn_connect.setEnabled(False)
        self.worker = TcpWorker()
        self.worker.configure(host, port)
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)
        self.worker.parsed_msg.connect(self._on_parsed_msg)
        self.worker.error.connect(self._on_error)
        self.worker.start()

    def on_disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker.wait(150)
            self.worker = None
        self._set_initial_disconnected_ui()

    def _on_connected(self):
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.lbl_conn_text.setText("CONECTADO")
        self._set_label_color_state(self.lbl_conn_text, "green")
        self._first_sync_done = False
        self.poll_status()
        self._poll_timer.start(100)
        self._update_controls_interlock()

    def _on_disconnected(self, reason):
        self._set_initial_disconnected_ui()

    def _on_error(self, err):
        self._set_initial_disconnected_ui()

    def send_enable_actuators(self):
        if self.btn_enable_actuators.isChecked():
            if not self.use_grbl_protocol:
                self.send_compact("CMD|enable_actuators")
            else:
                self.send_compact("$X")
            self.btn_enable_actuators.setEnabled(False)
            self.btn_enable_actuators.setText("Actuadores ON")
        self._update_controls_interlock()

    def on_stop_clicked(self):
        if not self.use_grbl_protocol:
            self.send_compact("CMD|emergency_stop")
        else:
            self.send_compact("!")
        self.btn_enable_actuators.setEnabled(True)
        self.btn_enable_actuators.setChecked(False)
        self.btn_enable_actuators.setText("Activar Actuadores")
        if self.sequence_widget.is_running:
            self.sequence_widget._stop_sequence()
        self._update_controls_interlock()

    def send_single_home(self, ax: str):
        tab = self.axis_tabs[ax]
        blk = self.axis_blocks[ax]
        bo = int(tab.sp_backoff_steps.value())
        so = int(tab.sp_soft_offset_steps.value())
        if not self.use_grbl_protocol:
            self.send_compact(f"CMD|home_axis|{ax}|{bo}|{so}")
        else:
            self.send_compact(f"$H{ax.upper()}")

    def start_all_homing(self):
        if not self.use_grbl_protocol:
            self._homing_queue = ["x", "y", "z", "w"]
            self.send_single_home(self._homing_queue.pop(0))
            self._homing_timer.start(250)
        else:
            self.send_compact("$H")

    def _process_homing_queue(self):
        machine_state = str(self.last_status.get("machine_state", "")).lower()
        if machine_state != "homing" and len(self._homing_queue) > 0:
            next_ax = self._homing_queue.pop(0)
            self.send_single_home(next_ax)
        elif len(self._homing_queue) == 0 and machine_state != "homing":
            self._homing_timer.stop()

    def send_homing_vars(self):
        ax = self._axis_from_tab()
        tab = self.axis_tabs[ax]
        blk = self.axis_blocks[ax]
        mt = float(tab.sp_max_travel.value())
        seek_us = int(tab.sp_hseek.value())
        feed_us = int(tab.sp_hfeed.value())
        bo_us = int(tab.sp_hbo.value())
        self.send_compact(f"CMD|set_homing_speed_axis|{ax}|{seek_us}|{feed_us}|{bo_us}")
        
        fecha_calib = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.send_compact(f"CMD|set_calibration_axis|{ax}|{float(tab.current_spm):.4f}|{mt:.2f}|{fecha_calib}")
        tab.lbl_last_calib_date.setText(fecha_calib)

    def poll_status(self):
        if not self.use_grbl_protocol:
            self.send_compact("GET_STATUS")
        else:
            self.send_compact("?")

    def _check_target_inside_soft_limit_for_axis(self, axis_name: str, target: float):
        try:
            soft_limit_max = max(0.0, float(self.axis_tabs[axis_name].sp_max_travel.value()))
        except Exception:
            soft_limit_max = 110.0
        if target < -0.05 or target > (soft_limit_max + 0.05):
            QMessageBox.warning(self, "Límites", f"Eje {axis_name.upper()}: Objetivo {target:.2f}mm fuera de rango (Max {soft_limit_max:.2f}mm).")
            return False
        return True

    def on_set_profile_and_speed(self, axis, tab):
        s = float(tab.sp_start_mms.value())
        c = float(tab.sp_cruise_mms.value())
        e = float(tab.sp_end_mms.value())
        r = float(tab.sp_ramp_ratio.value())
        self.send_compact(f"CMD|set_scurve_profile_axis|{axis}|{s:.2f}|{c:.2f}|{e:.2f}|{r:.2f}")
        self.send_compact(f"CMD|set_manual_speed_axis|{axis}|{int(tab.sp_manual_us.value())}|{int(tab.sp_jog_us.value())}")

    def _on_tab_changed(self, idx):
        ax_key = self._axis_from_tab()
        self._sync_tab_from_payload(ax_key, self.last_status)
        self._update_controls_interlock()
        self._update_status_ui(self.last_status or {})

    def _sync_tab_from_payload(self, ax_key, payload):
        if not payload: return
        d = payload.get("axes", {}).get(ax_key, {})
        if d:
            tab = self.axis_tabs[ax_key]
            blk = self.axis_blocks[ax_key]
            if "max_travel" in d: tab.sp_max_travel.setValue(float(d["max_travel"]))
            if "home_seek_us" in d: tab.sp_hseek.setValue(int(d["home_seek_us"]))
            if "home_feed_us" in d: tab.sp_hfeed.setValue(int(d["home_feed_us"]))
            if "home_backoff_us" in d: tab.sp_hbo.setValue(int(d["home_backoff_us"]))
            if "steps_per_mm" in d: tab.current_spm = float(d["steps_per_mm"])
            if "backoff_steps" in d: tab.sp_backoff_steps.setValue(int(d["backoff_steps"]))
            if "soft_offset_steps" in d: tab.sp_soft_offset_steps.setValue(int(d["soft_offset_steps"]))
            if "use_scurve" in d: 
                blk.chk_scurve.blockSignals(True)
                blk.chk_scurve.setChecked(bool(d["use_scurve"]))
                blk.chk_scurve.blockSignals(False)
            if "last_calibration" in d and d["last_calibration"]:
                tab.lbl_last_calib_date.setText(str(d["last_calibration"]))

            sc = d.get("scurve_profile", {})
            if sc:
                tab.sp_start_mms.setValue(float(sc.get("start_mm_s", 1.5)))
                tab.sp_cruise_mms.setValue(float(sc.get("cruise_mm_s", 15.0)))
                tab.sp_end_mms.setValue(float(sc.get("end_mm_s", 1.5)))
                tab.sp_ramp_ratio.setValue(float(sc.get("ramp_ratio", 0.25)))
            
            if "manual_us" in d: tab.sp_manual_us.setValue(int(d["manual_us"]))
            if "jog_us" in d: tab.sp_jog_us.setValue(int(d["jog_us"]))

    def _sync_ui_from_payload(self, payload):
        for ax in self.AXES: self._sync_tab_from_payload(ax, payload)
        self._sync_tab_from_payload(self._axis_from_tab(), payload)

    def _set_machine_state_label(self, state: str):
        txt = (state or "-").upper()
        self.lbl_machine_state_v.setText(txt)
        s = txt.lower()
        if s == "idle": c = "green"
        elif s in ("running", "manual"): c = "blue"
        elif s in ("homing", "calibrating"): c = "yellow"
        elif s == "alarm": c = "red"
        else: c = "default"
        self._set_label_color_state(self.lbl_machine_state_v, c)

    def _update_status_ui(self, payload: dict):
        if not payload: return
        if not self._first_sync_done:
            self._sync_ui_from_payload(payload)
            self._first_sync_done = True

        self._set_machine_state_label(str(payload.get("machine_state", "-")))
        act_ok = bool(payload.get("actuators_enabled", False))
        self.lbl_actuators_text.setText("HABILITADOS" if act_ok else "DESHABILITADOS")
        self._set_label_color_state(self.lbl_actuators_text, "green" if act_ok else "red")
        
        if act_ok != self.btn_enable_actuators.isChecked():
            self.btn_enable_actuators.blockSignals(True)
            self.btn_enable_actuators.setChecked(act_ok)
            self.btn_enable_actuators.setText("Actuadores ON" if act_ok else "Activar Actuadores")
            self.btn_enable_actuators.setEnabled(not act_ok)
            self.btn_enable_actuators.blockSignals(False)

        axes_data = payload.get("axes", {}) if isinstance(payload.get("axes"), dict) else {}

        for a in self.AXES:
            d = axes_data.get(a, {})
            self.axis_blocks[a].update_data(d)
            if "step_count" in d:
                self.axis_tabs[a].set_steps_count(int(d["step_count"]))
            if "last_calibration" in d and d["last_calibration"]:
                self.axis_tabs[a].lbl_last_calib_date.setText(str(d["last_calibration"]))

        self._update_controls_interlock()

    def _on_parsed_msg(self, msg):
        try:
            mtype = msg.get("type", "")
            payload = msg.get("payload", {}) or {}

            if mtype == "status":
                self.last_status = payload
                self._update_status_ui(payload)
                self.last_machine_state = str(payload.get('machine_state', '-'))
            elif mtype == "grbl_status":
                raw = str(payload).strip("<>")
                parts = raw.split("|")
                if len(parts) >= 2:
                    st = parts[0].lower()
                    self._set_machine_state_label(st)
                    for item in parts[1:]:
                        if item.startswith("WPos:") or item.startswith("MPos:"):
                            coords = item.split(":")[1].split(",")
                            for idx, ax_char in enumerate(["x", "y", "z", "w"]):
                                if idx < len(coords):
                                    val = float(coords[idx])
                                    self.axis_blocks[ax_char].lbl_pos_num.setText(f"{val:.3f} mm")
            elif mtype == "ack":
                pass
        except Exception: pass

    def closeEvent(self, event):
        if self._poll_timer.isActive():
            self._poll_timer.stop()
        if self._deadman_timer.isActive():
            self._deadman_timer.stop()
        if self.sequence_widget.is_running:
            self.sequence_widget._stop_sequence()
        self._dpad_release_group("xy"); self._dpad_release_group("zw")
        if self.cam_widget:
            self.cam_widget.stop_camera()
        try:
            if self.worker: self.worker.stop(); self.worker.wait(200)
        except Exception: pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()