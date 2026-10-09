"""屏幕适配逻辑；控件、布局、配色和基准字号均由 main_window.ui 定义。"""

import hashlib

from PyQt5 import QtCore, QtGui, QtWidgets


def configure_high_dpi():
    """在创建 QApplication 前调用；系统缩放只交给 Qt 处理一次。"""
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    QtGui.QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
        QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)


def automatic_scale(available_size):
    """输入为逻辑像素：4K/200% 约 1920×1080，2K/100% 约 2560×1440。"""
    return max(0.85, min(1.6, available_size.width() / 1920,
                        available_size.height() / 1080))


class WindowDisplayController(QtCore.QObject):
    SCALE_MULTIPLIERS = (1.0, 0.9, 1.1, 1.25)

    def __init__(self, window, ui, settings=None):
        super().__init__(window)
        self.window = window
        self.ui = ui
        self.settings = (settings if settings is not None
                         else QtCore.QSettings('URControl', 'Display'))
        self._screen = None
        self._available = None
        self._updating = False
        self._initial_size = QtCore.QSize(window.size())
        self.classic = hasattr(ui, 'scrollArea_Classic')
        self._connections = []
        if self.classic:
            self._canvas_size = QtCore.QSize(ui.classicCanvas.minimumSize())
            self._fixed_rects = [(widget, QtCore.QRect(widget.geometry()))
                                 for widget in vars(ui).values()
                                 if isinstance(widget, QtWidgets.QWidget)
                                 and ui.classicCanvas.isAncestorOf(widget)
                                 and widget.parentWidget().layout() is None]
        # QSS 会缓存子控件字体，仅改变窗口字体不能可靠地传播字号。
        # 保存 .ui 中每个控件解析后的基准字体，跨屏时始终从基准计算。
        self._fonts = [(window, QtGui.QFont(window.font()))]
        self._fonts.extend((child, QtGui.QFont(child.font()))
                           for child in window.findChildren(QtWidgets.QWidget))
        self._connect(self.ui.comboBox_UiScale.currentIndexChanged, self._on_scale_changed)
        self._connect(self.ui.checkBox_FTSwitch.toggled, self._update_force_labels)
        self._update_force_labels(self.ui.checkBox_FTSwitch.isChecked(), clear_values=False)
        self._unread_logs = 0
        if not self.classic:
            self._log_tab = self.ui.tabWidget_Telemetry.indexOf(self.ui.tab_Log)
            self._log_title = self.ui.tabWidget_Telemetry.tabText(self._log_tab)
            self._connect(self.ui.plainTextEdit_DashboardMessage.textChanged, self._on_log_changed)
            self._connect(self.ui.tabWidget_Telemetry.currentChanged, self._on_telemetry_changed)
            self.ui.splitter_Main.setStretchFactor(0, 1)
            self.ui.splitter_Main.setStretchFactor(1, 1)
            self.ui.splitter_Views.setStretchFactor(0, 3)
            self.ui.splitter_Views.setStretchFactor(1, 2)
            self.ui.splitter_Cameras.setStretchFactor(0, 1)
            self.ui.splitter_Cameras.setStretchFactor(1, 1)

        # 在鼠标所在显示器打开；之后跟随 QWindow 的实际屏幕变化。
        screen = (QtGui.QGuiApplication.screenAt(QtGui.QCursor.pos())
                  or QtGui.QGuiApplication.primaryScreen())
        window.winId()
        self._handle = window.windowHandle()
        self._handle.setScreen(screen)
        self._connect(self._handle.screenChanged, self._on_screen_changed)
        self._on_screen_changed(screen)
        if not self.classic:
            self.ui.splitter_Main.setSizes([int(window.width() * 0.52),
                                           int(window.width() * 0.48)])
            self.ui.splitter_Views.setSizes([540, 320])
            self.ui.splitter_Cameras.setSizes([1, 1])

    def _connect(self, signal, slot):
        signal.connect(slot)
        self._connections.append((signal, slot))

    def dispose(self):
        """切换布局前解除显示层信号，业务控件的信号连接保持不变。"""
        for signal, slot in self._connections:
            signal.disconnect(slot)
        if self._screen is not None:
            self._screen.availableGeometryChanged.disconnect(self._on_metrics_changed)
            self._screen.logicalDotsPerInchChanged.disconnect(self._on_metrics_changed)
        self.deleteLater()

    def _settings_key(self):
        identity = '|'.join((self._screen.name(), self._screen.manufacturer(),
                             self._screen.model(), self._screen.serialNumber()))
        key = hashlib.sha256(identity.encode('utf-8')).hexdigest()[:16]
        mode = 'classic' if self.classic else 'modern'
        return f'screens/{key}/{mode}/scaleIndex'

    def _on_screen_changed(self, screen):
        if screen is None or screen is self._screen:
            return
        if self._screen is not None:
            self._screen.availableGeometryChanged.disconnect(self._on_metrics_changed)
            self._screen.logicalDotsPerInchChanged.disconnect(self._on_metrics_changed)
        self._screen = screen
        screen.availableGeometryChanged.connect(self._on_metrics_changed)
        screen.logicalDotsPerInchChanged.connect(self._on_metrics_changed)
        try:
            key = self._settings_key()
            if not self.classic and not self.settings.contains(key):
                # 沿用尚未区分界面模式时保存的新版字号偏好。
                key = key.replace('/modern/', '/')
            index = self.settings.value(key, 0, type=int)
        except (TypeError, ValueError):
            index = 0
        if not 0 <= index < len(self.SCALE_MULTIPLIERS):
            index = 0
        blocker = QtCore.QSignalBlocker(self.ui.comboBox_UiScale)
        self.ui.comboBox_UiScale.setCurrentIndex(index)
        del blocker
        self._apply_display(fit_window=True)

    def _on_metrics_changed(self, *_):
        self._apply_display(fit_window=True)

    def _on_scale_changed(self, index):
        if self._screen is None:
            return
        self.settings.setValue(self._settings_key(), index)
        self._apply_display(fit_window=False)

    def _apply_display(self, *, fit_window):
        if self._updating or self._screen is None:
            return
        self._updating = True
        try:
            available = self._screen.availableGeometry()
            index = self.ui.comboBox_UiScale.currentIndex()
            scale = automatic_scale(available.size()) * self.SCALE_MULTIPLIERS[index]
            for widget, base_font in self._fonts:
                font = QtGui.QFont(base_font)
                if base_font.pointSizeF() > 0:
                    font.setPointSizeF(base_font.pointSizeF() * scale)
                else:
                    font.setPixelSize(max(1, round(base_font.pixelSize() * scale)))
                widget.setFont(font)
            if self.classic:
                self.ui.classicCanvas.setMinimumSize(
                    round(self._canvas_size.width() * scale),
                    round(self._canvas_size.height() * scale))
                for widget, rect in self._fixed_rects:
                    widget.setGeometry(round(rect.x() * scale), round(rect.y() * scale),
                                       round(rect.width() * scale), round(rect.height() * scale))
            self.ui.comboBox_UiScale.setToolTip(
                f'{self._screen.name()} · 可用区域 {available.width()} × '
                f'{available.height()} 逻辑像素\n'
                f'当前基准字号 {self.window.font().pointSizeF():.1f} pt；'
                '百分比相对自动适配大小，按显示器记忆。')
            if fit_window and not (self.window.isMaximized() or self.window.isFullScreen()):
                if self._available is None:
                    auto = automatic_scale(available.size())
                    width = min(round(self._initial_size.width() * auto),
                                round(available.width() * 0.82))
                    height = min(round(self._initial_size.height() * auto),
                                 round(available.height() * 0.82))
                else:
                    width_ratio = min(0.96, self.window.width() / self._available.width())
                    height_ratio = min(0.94, self.window.height() / self._available.height())
                    width = round(available.width() * width_ratio)
                    height = round(available.height() * height_ratio)
                self.window.resize(width, height)
                # 包含窗口装饰边框，避免初次打开或跨屏后超出工作区。
                frame = self.window.frameGeometry()
                if self._available is None:
                    frame.moveCenter(available.center())
                x = max(available.left(), min(frame.x(), available.right() - frame.width() + 1))
                y = max(available.top(), min(frame.y(), available.bottom() - frame.height() + 1))
                self.window.move(x, y)
            self._available = QtCore.QRect(available)
        finally:
            self._updating = False

    def _update_force_labels(self, joint_torques, *, clear_values=True):
        # 经典布局的标题旁紧贴“力矩”复选框，保留原来的短标题。
        self.ui.groupBox_Fex.setTitle('力' if self.classic else
                                     ('关节力矩' if joint_torques else 'TCP 广义力'))
        labels = ([f'J{i} · N·m' for i in range(1, 7)] if joint_torques else
                  ['Fx · N', 'Fy · N', 'Fz · N', 'Mx · N·m', 'My · N·m', 'Mz · N·m'])
        for i, text in enumerate(labels, 1):
            if not self.classic:
                getattr(self.ui, f'label_ForceAxis{i}').setText(text)
            # 切换数据源后等待新样本，避免旧数值短暂显示在新单位下。
            if clear_values:
                getattr(self.ui, f'lineEdit_Fex{i}').clear()

    def _on_log_changed(self):
        if (self.ui.tabWidget_Telemetry.currentIndex() == self._log_tab
                or self.ui.plainTextEdit_DashboardMessage.document().isEmpty()):
            self._on_telemetry_changed(self._log_tab)
            return
        self._unread_logs += 1
        count = str(self._unread_logs) if self._unread_logs < 100 else '99+'
        self.ui.tabWidget_Telemetry.setTabText(self._log_tab, f'{self._log_title} ({count})')

    def _on_telemetry_changed(self, index):
        if index == self._log_tab:
            self._unread_logs = 0
            self.ui.tabWidget_Telemetry.setTabText(self._log_tab, self._log_title)
