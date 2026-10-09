"""Qt Designer 提升控件：相机画面随布局缩放，并保持原始宽高比。"""

from PyQt5 import QtCore, QtGui, QtWidgets


class AspectRatioLabel(QtWidgets.QLabel):
    """绘制原始帧，避免 QLabel 的 scaledContents 把横屏/竖屏画面拉伸。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._frame = QtGui.QPixmap()

    def setPixmap(self, pixmap):
        self._frame = QtGui.QPixmap(pixmap)
        super().setText('')
        self.update()

    def pixmap(self):
        return QtGui.QPixmap(self._frame)

    def setText(self, text):
        self._frame = QtGui.QPixmap()
        super().setText(text)

    def clear(self):
        self._frame = QtGui.QPixmap()
        super().clear()

    def sizeHint(self):
        # 不把视频帧分辨率当作控件的最小尺寸。
        return QtCore.QSize(320, 180)

    def minimumSizeHint(self):
        return QtCore.QSize(140, 120)

    def paintEvent(self, event):
        super().paintEvent(event)
        if self._frame.isNull():
            return
        area = self.contentsRect()
        size = self._frame.size().scaled(area.size(), QtCore.Qt.KeepAspectRatio)
        target = QtCore.QRect(QtCore.QPoint(), size)
        target.moveCenter(area.center())
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        painter.drawPixmap(target, self._frame)
