"""Designer 提升的只读趋势图；只绘制有限缓存，不创建通信或运动通道。"""

from copy import deepcopy
import math

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import QWidget


class AdmittanceTrendWidget(QWidget):
    """同一物理单位共用纵轴，按运行与有效性断开折线。"""

    cursor_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._data = {"title": "等待显示样本", "unit": "", "series": [], "now": 0.0}
        self._window = 15.0
        self._paused = False
        self._cursor_x = None
        self._plot_rect = QRectF()
        self.setMouseTracking(True)
        self.setMinimumHeight(240)

    def set_data(self, data):
        """暂停时保留当前显示；后台缓存仍可继续接收样本。"""
        if not self._paused:
            self._data = data
            self.update()

    def set_time_window(self, seconds):
        if seconds not in (5, 15, 30, 60):
            raise ValueError("显示时间窗只支持 5、15、30、60 秒")
        self._window = float(seconds)
        self.update()

    def set_paused(self, paused):
        paused = bool(paused)
        if paused and not self._paused:
            self._data = deepcopy(self._data)
        self._paused = paused
        self.update()

    def clear(self):
        """清除本控件的显示数据，不影响监视器或采集器。"""
        self._data = dict(self._data, series=[])
        self._cursor_x = None
        self.cursor_changed.emit("")
        self.update()

    def mouseMoveEvent(self, event):
        self._cursor_x = event.pos().x() if self._plot_rect.contains(QPointF(event.pos())) else None
        self.update()

    def leaveEvent(self, event):
        self._cursor_x = None
        self.cursor_changed.emit("")
        self.update()

    def paintEvent(self, event):
        """采用时间戳定位，光标只查找实际样本，不在断点间插值。"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setPen(QColor("#263448"))
        title = self._data["title"] + ("（显示已暂停）" if self._paused else "")
        painter.drawText(QRectF(12, 6, self.width() - 24, 24), Qt.AlignLeft | Qt.AlignVCenter, title)
        x, legend_y = 14, 40
        for series in self._data["series"]:
            width = painter.fontMetrics().horizontalAdvance(series["name"]) + 42
            if x + width > self.width() - 12 and x > 14:
                x, legend_y = 14, legend_y + 22
            pen = QPen(QColor(series["color"]), 2)
            pen.setStyle({"solid": Qt.SolidLine, "dash": Qt.DashLine, "dot": Qt.DotLine}[series["style"]])
            painter.setPen(pen)
            painter.drawLine(QPointF(x, legend_y), QPointF(x + 24, legend_y))
            painter.setPen(QColor("#263448"))
            painter.drawText(QPointF(x + 30, legend_y + 4), series["name"])
            x += width
        top = legend_y + 24
        rect = QRectF(80, top, max(20, self.width() - 100), max(30, self.height() - top - 42))
        self._plot_rect = rect
        now = self._data["now"]
        earliest = now - self._window
        visible_series = []
        y_values = []
        for series in self._data["series"]:
            points = [p for p in series["points"] if earliest <= p["time"] <= now]
            visible_series.append((series, points))
            y_values.extend(p["value"] for p in points if p["valid"] and math.isfinite(p["value"]))
        low, high = (min(y_values), max(y_values)) if y_values else (-1.0, 1.0)
        margin = max((high - low) * 0.1, abs(high) * 0.02, 0.001)
        low, high = low - margin, high + margin

        def point_xy(point):
            return QPointF(rect.left() + (point["time"] - earliest) / self._window * rect.width(),
                           rect.bottom() - (point["value"] - low) / (high - low) * rect.height())

        painter.setPen(QColor("#607084"))
        painter.drawText(QRectF(4, top - 22, 72, 22), Qt.AlignRight | Qt.AlignVCenter, self._data["unit"])
        for index in range(5):
            fraction = index / 4
            y = rect.bottom() - fraction * rect.height()
            painter.setPen(QColor("#e6eaf0"))
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            painter.setPen(QColor("#607084"))
            painter.drawText(QRectF(2, y - 10, 70, 20), Qt.AlignRight | Qt.AlignVCenter,
                             f"{low + fraction * (high - low):.4g}")
            x = rect.left() + fraction * rect.width()
            painter.setPen(QColor("#e6eaf0"))
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            painter.setPen(QColor("#607084"))
            painter.drawText(QRectF(x - 28, rect.bottom() + 8, 56, 20), Qt.AlignCenter,
                             f"{(fraction - 1) * self._window:.0f} s")
        painter.setPen(QPen(QColor("#a5b0c0"), 1))
        painter.drawRect(rect)
        painter.save()
        painter.setClipRect(rect.adjusted(-1, -1, 1, 1))
        for series, points in visible_series:
            pen = QPen(QColor(series["color"]), 1.7)
            pen.setStyle({"solid": Qt.SolidLine, "dash": Qt.DashLine, "dot": Qt.DotLine}[series["style"]])
            painter.setPen(pen)
            path = QPainterPath()
            last_identity = None
            for point in points:
                if not point["valid"] or not math.isfinite(point["value"]):
                    last_identity = None
                    continue
                identity = (point["run_id"], point["segment"])
                xy = point_xy(point)
                if identity != last_identity:
                    path.moveTo(xy)
                    painter.drawEllipse(xy, 1.5, 1.5)
                else:
                    path.lineTo(xy)
                last_identity = identity
            painter.drawPath(path)
        if self._cursor_x is not None:
            painter.setPen(QPen(QColor("#64748b"), 1, Qt.DashLine))
            painter.drawLine(QPointF(self._cursor_x, rect.top()), QPointF(self._cursor_x, rect.bottom()))
            cursor_time = earliest + (self._cursor_x - rect.left()) / rect.width() * self._window
            readings = [f"光标相对当前 {cursor_time - now:.3f} s"]
            tolerance = self._window * 8.0 / rect.width()
            for series, points in visible_series:
                candidates = [p for p in points if p["valid"] and math.isfinite(p["value"])]
                if candidates:
                    nearest = min(candidates, key=lambda p: abs(p["time"] - cursor_time))
                    if abs(nearest["time"] - cursor_time) <= tolerance:
                        readings.append(f"{series['name']} {nearest['value']:.5g} {self._data['unit']}（本轮 {nearest['run_id']}）")
                        painter.setPen(QPen(QColor(series["color"]), 2))
                        painter.drawEllipse(point_xy(nearest), 3, 3)
            self.cursor_changed.emit("；".join(readings))
        painter.restore()
        if not y_values:
            painter.setPen(QColor("#7b8798"))
            painter.drawText(rect, Qt.AlignCenter, "当前时间窗无有效样本")
