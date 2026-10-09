"""仅预览界面，不导入硬件模块、不连接设备、不创建采集文件。"""

import sys

from PyQt5 import QtCore, QtWidgets

from window_display import WindowDisplayController, configure_high_dpi
from window_layouts import create_layout, saved_layout, switch_layout


class PreviewWindow(QtWidgets.QMainWindow):
    def __init__(self, settings=None):
        super().__init__()
        self.settings = (settings if settings is not None
                         else QtCore.QSettings('URControl', 'Display'))
        self.mode = saved_layout(self.settings)
        self.ui = create_layout(self, self.mode)
        self.display = WindowDisplayController(self, self.ui, self.settings)
        self._configure_preview()

    def _configure_preview(self):
        self.setWindowTitle('URControl · 离线界面预览')
        label = (self.ui.label_ClassicTitle if self.display.classic
                 else self.ui.label_Subtitle)
        label.setText('离线界面预览 · 不连接设备')
        self.ui.actionLayoutModern.triggered.connect(lambda: self.change_layout('modern'))
        self.ui.actionLayoutClassic.triggered.connect(lambda: self.change_layout('classic'))

    def change_layout(self, mode):
        if mode == self.mode:
            return
        self.setUpdatesEnabled(False)
        try:
            self.display.dispose()
            self.ui = switch_layout(self, self.ui, mode)
            self.mode = mode
            self.settings.setValue('layoutMode', mode)
            self.display = WindowDisplayController(self, self.ui, self.settings)
            self._configure_preview()
        finally:
            self.setUpdatesEnabled(True)


if __name__ == '__main__':
    configure_high_dpi()
    app = QtWidgets.QApplication(sys.argv)
    window = PreviewWindow()
    window.show()
    sys.exit(app.exec_())
