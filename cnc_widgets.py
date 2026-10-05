"""
Widgets de Control Vectorial, Visor Óptico y Paneles DRO - CNC XYZW
Versión: 6.5.1 (Pure GRBL Engine)
Archivo: cnc_widgets.py
"""

import sys
import math
import cv2

from PySide6.QtCore import Qt, QThread, Signal, QPointF, QRectF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPainterPath, QImage
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QDoubleSpinBox, QSpinBox, QGroupBox, QDialog, QFormLayout, QMessageBox,
    QSizePolicy, QCheckBox, QLineEdit, QSlider
)


class ToggleSwitch(QWidget):
    toggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(44, 18)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
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
        h = r.height()
        radius = h / 2.0
        p.drawRoundedRect(r, radius, radius)

        diameter = h - 4
        thumb_x = r.width() - diameter - 2 if self._checked else 2
        p.setBrush(QColor("#ffffff"))
        p.drawEllipse(thumb_x, 2, diameter, diameter)
        p.end()


class CameraWorker(QThread):
    frame_ready = Signal(object)

    def __init__(self, index=0, parent=None):
        super().__init__(parent)
        self.index = index
        self._running = False

    def run(self):
        backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
        cap = cv2.VideoCapture(self.index, backend)
        if not cap or not cap.isOpened():
            self.frame_ready.emit(None)
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._running = True
        while self._running:
            ret, frame = cap.read()
            if ret:
                self.frame_ready.emit(frame)
            else:
                self.msleep(30)
            self.msleep(15)
        cap.release()

    def stop(self):
        self._running = False


class CameraMeasurementWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(240, 150)
        self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.cam_worker = None
        self.cam_opening = False
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

    def camera_running(self) -> bool:
        return bool(self.cam_worker and self.cam_worker.isRunning() and self.cam_worker._running)

    def start_camera(self, index=0):
        self.stop_camera()
        self.cam_opening = True
        self.cam_worker = CameraWorker(index, self)
        self.cam_worker.frame_ready.connect(self._on_frame)
        self.cam_worker.start()

    def _on_frame(self, frame):
        self.cam_opening = False
        if frame is None:
            self.stop_camera()
            self.raw_frame = None
        else:
            self.raw_frame = frame
        self.update()

    def stop_camera(self):
        if self.cam_worker:
            self.cam_worker.stop()
            if self.cam_worker.isRunning():
                if not self.cam_worker.wait(1500):
                    self.cam_worker.terminate()
                    self.cam_worker.wait(300)
            self.cam_worker = None
        self.cam_opening = False
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

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w_img, h_img, ch * w_img, QImage.Format_RGB888)
        p.drawImage(target_rect, qimg)

        if self.grid_enabled and self.grid_size > 4:
            p.save()
            p.setClipRect(target_rect)
            p.setPen(QPen(QColor(57, 255, 20, 140), 1, Qt.DashLine))
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

        all_l = list(self.measurement_lines)
        if self.measuring and self.p1 and self.p2:
            all_l.append((self.p1, self.p2))

        def draw_cross_marker(painter, pt, size=5):
            painter.drawLine(QPointF(pt.x() - size, pt.y()), QPointF(pt.x() + size, pt.y()))
            painter.drawLine(QPointF(pt.x(), pt.y() - size), QPointF(pt.x(), pt.y() + size))

        for pt1, pt2 in all_l:
            p.setPen(QPen(QColor("#ff0055"), 2))
            p.drawLine(pt1, pt2)
            p.setPen(QPen(QColor("#00f0ff"), 2))
            draw_cross_marker(p, pt1)
            draw_cross_marker(p, pt2)

            dx = pt2.x() - pt1.x()
            dy = pt2.y() - pt1.y()
            dist_px = math.hypot(dx, dy)
            dist_frame_px = (dist_px / (scale if scale > 0 else 1.0)) / self.zoom_factor
            dist_mm = dist_frame_px / (self.px_per_mm if self.px_per_mm > 0 else 1.0)

            mid = (pt1 + pt2) / 2.0
            p.setFont(QFont("Consolas", 10, QFont.Bold))
            text_rect = QRectF(mid.x() - 45, mid.y() - 10, 90, 20)
            p.setPen(QColor("#000000"))
            p.drawText(text_rect.translated(1, 1), Qt.AlignCenter, f"{dist_mm:.3f} mm")
            p.setPen(QColor("#00f0ff"))
            p.drawText(text_rect, Qt.AlignCenter, f"{dist_mm:.3f} mm")

        p.end()


class DPadPainter(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)
    stopPressed = Signal()

    def __init__(self, labels=None, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.labels = labels or {"up": "Y+", "down": "Y-", "left": "X-", "right": "X+"}
        self._pressed = False
        self._active = None

    def _geom(self):
        w, h = self.width(), self.height()
        dim = min(w, h)
        c = QPointF(w / 2.0, h / 2.0)
        outer = dim * 0.47
        inner = dim * 0.23
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
        if zone:
            self._pressed = True
            self._active = zone
            if zone == "stop": self.stopPressed.emit()
            else: self.directionPressed.emit(zone)
            self.update()

    def mouseMoveEvent(self, e):
        if not self.isEnabled() or not self._pressed: return
        zone = self._hit(e.position())
        if zone in ("up", "down", "left", "right", "stop") and zone != self._active:
            old = self._active
            self._active = zone
            if old and old != "stop": self.directionReleased.emit(old)
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
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c, outer, inner, stop_r = self._geom()
        p.fillRect(self.rect(), QColor("#161b22"))
        p.setPen(QPen(QColor("#2f4953"), max(1, int(outer * 0.015))))
        p.setBrush(QColor("#0f1a20"))
        p.drawEllipse(c, outer, outer)
        gap = max(2.5, outer * 0.035)

        def draw_sec(start_deg, span_deg, key):
            out_r = QRectF(c.x() - outer, c.y() - outer, outer * 2, outer * 2)
            in_r = QRectF(c.x() - inner, c.y() - inner, inner * 2, inner * 2)
            pth = QPainterPath()
            pth.arcMoveTo(out_r, start_deg + gap / 2.0)
            pth.arcTo(out_r, start_deg + gap / 2.0, span_deg - gap)
            pth.arcTo(in_r, start_deg + span_deg - gap / 2.0, -(span_deg - gap))
            pth.closeSubpath()
            p.setPen(QPen(QColor("#2f4953"), max(1, int(outer * 0.015))))
            p.setBrush(QColor("#1f4958") if self._active == key else (QColor("#12232c") if self.isEnabled() else QColor("#1c2128")))
            p.drawPath(pth)

        draw_sec(45, 90, "up")
        draw_sec(135, 90, "left")
        draw_sec(225, 90, "down")
        draw_sec(315, 90, "right")

        p.setPen(QPen(QColor("#b02a2a"), max(2, int(stop_r * 0.06))))
        p.setBrush(QColor("#a42424") if self._active == "stop" else QColor("#7d1b1b"))
        p.drawEllipse(c, stop_r, stop_r)
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))

        f_size = max(9, int(inner * 0.38))
        p.setFont(QFont("Segoe UI", f_size, QFont.Bold))
        lbl_w = outer * 0.6
        lbl_h = max(16, int(inner * 0.5))
        p.drawText(QRectF(c.x() - lbl_w / 2, c.y() - outer + (outer - inner) * 0.18, lbl_w, lbl_h), Qt.AlignCenter, self.labels["up"])
        p.drawText(QRectF(c.x() - outer + (outer - inner) * 0.12, c.y() - lbl_h / 2, lbl_w, lbl_h), Qt.AlignCenter, self.labels["left"])
        p.drawText(QRectF(c.x() + inner + (outer - inner) * 0.28, c.y() - lbl_h / 2, lbl_w, lbl_h), Qt.AlignCenter, self.labels["right"])
        p.drawText(QRectF(c.x() - lbl_w / 2, c.y() + inner + (outer - inner) * 0.32, lbl_w, lbl_h), Qt.AlignCenter, self.labels["down"])

        p.setFont(QFont("Segoe UI", max(8, int(stop_r * 0.35)), QFont.Bold))
        p.drawText(QRectF(c.x() - stop_r, c.y() - stop_r, stop_r * 2, stop_r * 2), Qt.AlignCenter, "STOP")
        p.end()


class VerticalZWControl(QWidget):
    directionPressed = Signal(str)
    directionReleased = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(140, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self._pressed = False
        self._active = None

    def _get_outer_oval_rect(self):
        w, h = self.width(), self.height()
        oval_w = min(w * 0.84, h * 0.56)
        oval_h = min(h * 0.94, oval_w * 1.62)
        return QRectF((w - oval_w) / 2.0, (h - oval_h) / 2.0, oval_w, oval_h)

    def _get_inner_oval_rect(self, outer_r, gap):
        h_sec = (outer_r.height() - gap * 3) / 4.0
        top_w = outer_r.top() + h_sec + gap
        return QRectF(outer_r.left() + outer_r.width() * 0.08, top_w, outer_r.width() * 0.84, (h_sec * 2) + gap)

    def _hit(self, pos):
        outer_r = self._get_outer_oval_rect()
        if not outer_r.contains(pos): return None
        gap = max(2.5, outer_r.width() * 0.035)
        inner_r = self._get_inner_oval_rect(outer_r, gap)
        if inner_r.contains(pos): return "w_up" if pos.y() < inner_r.center().y() else "w_down"
        return "z_up" if pos.y() < outer_r.center().y() else "z_down"

    def mousePressEvent(self, e):
        if not self.isEnabled() or e.button() != Qt.LeftButton: return
        zone = self._hit(e.position())
        if zone:
            self._pressed = True
            self._active = zone
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
            self._pressed = False
            self._active = None
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#161b22"))
        outer_r = self._get_outer_oval_rect()
        gap = max(2.5, outer_r.width() * 0.035)
        inner_r = self._get_inner_oval_rect(outer_r, gap)

        bg_out = QPainterPath()
        bg_out.addRoundedRect(outer_r, outer_r.width() / 2, outer_r.width() / 2)
        bg_in = QPainterPath()
        bg_in.addRoundedRect(inner_r, inner_r.width() / 2, inner_r.width() / 2)
        p.setPen(QPen(QColor("#2f4953"), max(1, int(outer_r.width() * 0.015))))
        p.setBrush(QColor("#0f1a20"))
        p.drawPath(bg_out)

        color_idle = QColor("#12232c") if self.isEnabled() else QColor("#1c2128")
        color_act = QColor("#1f4958")
        h_sec = (outer_r.height() - gap * 3) / 4.0

        sec_z_up = QRectF(outer_r.left(), outer_r.top(), outer_r.width(), h_sec)
        path_z_up = QPainterPath(); path_z_up.addRect(sec_z_up)
        p.setBrush(color_act if self._active == "z_up" else color_idle)
        p.drawPath(path_z_up.intersected(bg_out))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", max(9, int(h_sec * 0.42)), QFont.Bold))
        p.drawText(sec_z_up, Qt.AlignCenter, "Z+")

        sec_z_down = QRectF(outer_r.left(), outer_r.top() + (h_sec + gap) * 3, outer_r.width(), h_sec)
        path_z_down = QPainterPath(); path_z_down.addRect(sec_z_down)
        p.setBrush(color_act if self._active == "z_down" else color_idle)
        p.drawPath(path_z_down.intersected(bg_out))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.drawText(sec_z_down, Qt.AlignCenter, "Z-")

        p.setPen(QPen(QColor("#00f0ff") if self.isEnabled() else QColor("#2f4953"), 1, Qt.DashLine))
        p.setBrush(QColor("#0d181e"))
        p.drawPath(bg_in)

        half_inner = (inner_r.height() - gap) / 2.0
        sec_w_up = QRectF(inner_r.left(), inner_r.top(), inner_r.width(), half_inner)
        path_w_up = QPainterPath(); path_w_up.addRect(sec_w_up)
        p.setBrush(color_act if self._active == "w_up" else color_idle)
        p.drawPath(path_w_up.intersected(bg_in))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.setFont(QFont("Segoe UI", max(8, int(half_inner * 0.46)), QFont.Bold))
        p.drawText(sec_w_up, Qt.AlignCenter, "W+")

        sec_w_down = QRectF(inner_r.left(), inner_r.top() + half_inner + gap, inner_r.width(), half_inner)
        path_w_down = QPainterPath(); path_w_down.addRect(sec_w_down)
        p.setBrush(color_act if self._active == "w_down" else color_idle)
        p.drawPath(path_w_down.intersected(bg_in))
        p.setPen(QColor("#e6edf3") if self.isEnabled() else QColor("#484f58"))
        p.drawText(sec_w_down, Qt.AlignCenter, "W-")
        p.end()


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
        lbl_pos_tag = QLabel("Pos:")
        lbl_pos_tag.setStyleSheet("font-size: 24px; font-weight: bold; color: #f2cc60;")

        self.lbl_pos_num = QLabel("0.000 mm")
        self.lbl_pos_num.setStyleSheet("font-size: 24px; font-weight: bold; color: #00f0ff;")
        pos_box.addWidget(lbl_pos_tag)
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

        lbl_lpos_title = QLabel("L.Pos:")
        lbl_lpos_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.lbl_lpos_val = QLabel("0.000")
        self.lbl_lpos_val.setStyleSheet("color: #d2a8ff; font-weight: bold; font-size: 14px;")

        lbl_woff_title = QLabel("WOff:")
        lbl_woff_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.lbl_woff = QLabel("0.000")
        self.lbl_woff.setStyleSheet("color: #7ee787; font-weight: bold; font-size: 14px;")

        lbl_dir_title = QLabel("Dir:")
        lbl_dir_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.lbl_dir = QLabel("STOP")
        self.lbl_dir.setStyleSheet("font-weight: bold; color: #8b949e; font-size: 14px;")

        row2.addWidget(lbl_lpos_title)
        row2.addWidget(self.lbl_lpos_val)
        row2.addSpacing(10)
        row2.addWidget(lbl_woff_title)
        row2.addWidget(self.lbl_woff)
        row2.addSpacing(10)
        row2.addWidget(lbl_dir_title)
        row2.addWidget(self.lbl_dir)
        main_lay.addLayout(row2)

        row3 = QHBoxLayout()
        row3.setContentsMargins(0, 0, 0, 0)
        row3.setSpacing(4)
        row3.setAlignment(Qt.AlignCenter)

        lbl_abs_title = QLabel("Abs:")
        lbl_abs_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.sp_abs = QDoubleSpinBox()
        self.sp_abs.setRange(-5000.0, 5000.0); self.sp_abs.setDecimals(3); self.sp_abs.setFixedWidth(72)
        self.sp_abs.setStyleSheet("font-size: 13px;")
        self.btn_abs = QPushButton("Abs")
        self.btn_abs.setFixedWidth(42)

        lbl_rel_title = QLabel("Rel:")
        lbl_rel_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.sp_rel = QDoubleSpinBox()
        self.sp_rel.setRange(0.001, 500.0); self.sp_rel.setDecimals(3); self.sp_rel.setValue(10.0); self.sp_rel.setFixedWidth(66)
        self.sp_rel.setStyleSheet("font-size: 13px;")
        self.btn_rel_neg = QPushButton("- Rel")
        self.btn_rel_neg.setFixedWidth(48)
        self.btn_rel_pos = QPushButton("+ Rel")
        self.btn_rel_pos.setFixedWidth(48)

        self.btn_bkpos = QPushButton("BKPos")
        self.btn_bkpos.setFixedWidth(54)

        row3.addWidget(lbl_abs_title)
        row3.addWidget(self.sp_abs)
        row3.addWidget(self.btn_abs)
        row3.addSpacing(4)
        row3.addWidget(lbl_rel_title)
        row3.addWidget(self.sp_rel)
        row3.addWidget(self.btn_rel_neg)
        row3.addWidget(self.btn_rel_pos)
        row3.addSpacing(4)
        row3.addWidget(self.btn_bkpos)
        main_lay.addLayout(row3)

        row4 = QHBoxLayout()
        row4.setContentsMargins(0, 0, 0, 0)
        row4.setSpacing(5)

        self.btn_zero = QPushButton("Set Zero")
        self.btn_zero.setFixedWidth(70)

        self.btn_home = QPushButton("HOMING")
        self.btn_home.setFixedWidth(75)

        lbl_err_title = QLabel("Error:")
        lbl_err_title.setStyleSheet("font-size: 13px; font-weight: bold;")
        self.lbl_error = QLabel("Sin error")
        self.lbl_error.setStyleSheet("color: #ff7b72; font-size: 13px; font-weight: bold;")

        row4.addWidget(self.btn_zero)
        row4.addWidget(self.btn_home)
        row4.addSpacing(6)
        row4.addWidget(lbl_err_title)
        row4.addWidget(self.lbl_error)
        row4.addStretch(1)
        main_lay.addLayout(row4)

        self.btn_abs.clicked.connect(self._on_move_abs)
        self.btn_rel_neg.clicked.connect(lambda: self._on_move_rel(-abs(float(self.sp_rel.value()))))
        self.btn_rel_pos.clicked.connect(lambda: self._on_move_rel(abs(float(self.sp_rel.value()))))
        self.btn_bkpos.clicked.connect(self._on_bkpos_clicked)
        self.btn_zero.clicked.connect(self._on_zero_clicked)
        self.btn_home.clicked.connect(self._on_home_clicked)

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

    def update_data_direct(self, wpos_val: float, mpos_val: float, is_moving: bool, dir_str: str, homed: bool, err_str: str = ""):
        if is_moving and not self.was_moving:
            self.last_start_pos = wpos_val
            self.lbl_lpos_val.setText(f"{self.last_start_pos:.3f}")
        self.was_moving = is_moving

        self.lbl_pos_num.setText(f"{wpos_val:.3f} mm")
        self.lbl_woff.setText(f"{mpos_val:.3f}")
        self.lbl_dir.setText(dir_str.upper() if is_moving else "STOP")
        self.lbl_dir.setStyleSheet("color: #58a6ff; font-weight: bold; font-size: 14px;" if is_moving else "color: #8b949e; font-weight: bold; font-size: 14px;")

        self.set_moving_state(is_moving)
        self.set_homed_state(homed)

        if homed:
            self.lbl_homing.setText("HOMED")
            self.lbl_homing.setStyleSheet("background-color: #1a472a; color: #3fb950; font-weight: bold; border-radius: 3px; padding: 2px; font-size: 12px;")
        else:
            self.lbl_homing.setText("NO HOME")
            self.lbl_homing.setStyleSheet("background-color: #551a1a; color: #ff7b72; font-weight: bold; border-radius: 3px; padding: 2px; font-size: 12px;")

        self.lbl_error.setText(err_str if err_str else "Sin error")

    def _on_move_abs(self):
        val = float(self.sp_abs.value())
        feed = self.main_window.sp_jog_feed.value()
        try:
            self.last_start_pos = float(self.lbl_pos_num.text().replace(" mm", ""))
        except Exception:
            self.last_start_pos = 0.0
        self.lbl_lpos_val.setText(f"{self.last_start_pos:.3f}")
        self.main_window.send_gcode(f"G90 G21 G1 {self.axis.upper()}{val:.3f} F{feed}")

    def _on_move_rel(self, delta):
        feed = self.main_window.sp_jog_feed.value()
        try:
            self.last_start_pos = float(self.lbl_pos_num.text().replace(" mm", ""))
        except Exception:
            self.last_start_pos = 0.0
        self.lbl_lpos_val.setText(f"{self.last_start_pos:.3f}")
        self.main_window.send_gcode(f"G91 G21 G1 {self.axis.upper()}{float(delta):.3f} F{feed}")

    def _on_bkpos_clicked(self):
        target = self.last_start_pos
        feed = self.main_window.sp_jog_feed.value()
        self.main_window.send_gcode(f"G90 G21 G1 {self.axis.upper()}{target:.3f} F{feed}")

    def _on_zero_clicked(self):
        self.main_window.send_gcode(f"G10 L20 P1 {self.axis.upper()}0")

    def _on_home_clicked(self):
        self.main_window.send_gcode("$H")


class CalibrationDialog(QDialog):
    def __init__(self, axis_char: str, main_win, parent=None):
        super().__init__(parent)
        self.axis = axis_char.upper()
        self.main_win = main_win
        self.setWindowTitle(f"Asistente Calibración 2 Pasadas - Eje {self.axis}")
        self.setMinimumWidth(390)

        self.calib_step = 0
        self.start_mpos = 0.0
        self.dist1 = 0.0
        self.spm1 = 0.0
        self.dist2 = 0.0
        self.spm2 = 0.0
        self.initial_spm = self.main_win.get_axis_spm(self.axis)

        lay = QVBoxLayout(self)
        gb = QGroupBox("Calibración Steps/mm ($100-$103)")
        form = QFormLayout(gb)

        self.lbl_cur_spm = QLabel(f"{self.initial_spm:.3f} steps/mm")
        self.lbl_cur_spm.setStyleSheet("font-weight: bold; color: #58a6ff;")

        self.lbl_inst = QLabel("Haga clic en Iniciar P1 para comenzar.")
        self.lbl_inst.setStyleSheet("color: #e6edf3;")
        self.lbl_inst.setWordWrap(True)

        self.sp_meas = QDoubleSpinBox()
        self.sp_meas.setRange(0.001, 5000.0)
        self.sp_meas.setDecimals(3)
        self.sp_meas.setValue(20.0)
        self.sp_meas.setEnabled(False)

        self.lbl_res = QLabel("---")
        self.lbl_res.setStyleSheet("font-weight: bold; color: #00f0ff; font-size: 13px;")

        self.btn_act = QPushButton("Iniciar P1 (100% Vel)")
        self.btn_act.setStyleSheet("background-color: #238636; color: white; font-weight: bold;")

        form.addRow("SPM Actual:", self.lbl_cur_spm)
        form.addRow("Instrucciones:", self.lbl_inst)
        form.addRow("Distancia Medida (mm):", self.sp_meas)
        form.addRow("Nuevo SPM:", self.lbl_res)
        form.addRow(self.btn_act)
        lay.addWidget(gb)

        self.btn_act.clicked.connect(self._step_wizard)

    def _step_wizard(self):
        ax_i = ["X", "Y", "Z", "W"].index(self.axis)
        cur_mpos = self.main_win.mpos[ax_i]

        if self.calib_step == 0:
            self.calib_step = 1
            self.start_mpos = cur_mpos
            self.sp_meas.setEnabled(True)
            self.btn_act.setText("Confirmar Pasada 1 (P1)")
            self.lbl_inst.setText(f"Mueva el eje {self.axis} con el Jog. Mida con calibre y coloque la distancia arriba.")
        elif self.calib_step == 1:
            cmd_d = abs(cur_mpos - self.start_mpos)
            self.dist1 = self.sp_meas.value()
            if cmd_d < 0.5 or self.dist1 <= 0.0:
                QMessageBox.warning(self, "Calibración", "Desplazamiento insuficiente.")
                return
            self.spm1 = self.initial_spm * (cmd_d / self.dist1)
            self.calib_step = 2
            self.start_mpos = cur_mpos
            self.btn_act.setText("Confirmar Pasada 2 (P2)")
            self.lbl_inst.setText(f"P1: {self.spm1:.3f} stp/mm. Realice un segundo avance del eje {self.axis} a velocidad moderada y confirme.")
        elif self.calib_step == 2:
            cmd_d = abs(cur_mpos - self.start_mpos)
            self.dist2 = self.sp_meas.value()
            if cmd_d < 0.5 or self.dist2 <= 0.0:
                QMessageBox.warning(self, "Calibración", "Desplazamiento insuficiente en P2.")
                return
            self.spm2 = self.initial_spm * (cmd_d / self.dist2)
            avg = (self.spm1 + self.spm2) / 2.0
            diff = abs(self.spm1 - self.spm2) / avg * 100.0

            if diff > 5.0:
                r = QMessageBox.question(self, "Discrepancia", f"Diferencia: {diff:.1f}% (>5%).\nP1: {self.spm1:.3f} | P2: {self.spm2:.3f}\n¿Aplicar promedio?", QMessageBox.Yes | QMessageBox.No)
                if r == QMessageBox.No:
                    self.reject()
                    return

            self.lbl_res.setText(f"{avg:.3f} steps/mm")
            param = 100 + ax_i
            self.main_win.send_gcode(f"${param}={avg:.3f}")
            QMessageBox.information(self, "Guardado", f"Parámetro ${param} guardado en NVS: {avg:.3f} steps/mm.")
            self.accept()