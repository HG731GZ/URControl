"""经典主界面的屏幕适配；控件、布局、配色和基准字号由 classic_window.ui 定义。"""

from urllib.parse import quote

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
        # 首次显示后窗口装饰和布局尺寸才稳定，再校准一次。
        self._fit_timer = QtCore.QTimer(self)
        self._fit_timer.setSingleShot(True)
        self._fit_timer.timeout.connect(self._apply_display)
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
        self.ui.comboBox_UiScale.currentIndexChanged.connect(self._on_scale_changed)
        self.ui.checkBox_FTSwitch.toggled.connect(self._clear_force_values)

        # 在鼠标所在显示器打开；之后跟随 QWindow 的实际屏幕变化。
        screen = (QtGui.QGuiApplication.screenAt(QtGui.QCursor.pos())
                  or QtGui.QGuiApplication.primaryScreen())
        window.winId()
        self._handle = window.windowHandle()
        self._handle.setScreen(screen)
        self._handle.screenChanged.connect(self._on_screen_changed)
        self._on_screen_changed(screen)

    def _settings_key(self):
        # 直接保存显示器字段，转义设置路径的分隔符，不计算设备哈希。
        fields = (('name', self._screen.name()),
                  ('manufacturer', self._screen.manufacturer()),
                  ('model', self._screen.model()),
                  ('serial', self._screen.serialNumber()))
        key = '|'.join(f'{name}={quote(value, safe="")}' for name, value in fields)
        return f'screens/{key}/classic/scaleIndex'

    def _on_screen_changed(self, screen):
        if screen is None or screen is self._screen:
            return
        if self._screen is not None:
            self._screen.availableGeometryChanged.disconnect(self._on_metrics_changed)
            self._screen.logicalDotsPerInchChanged.disconnect(self._on_metrics_changed)
        self._screen = screen
        screen.availableGeometryChanged.connect(self._on_metrics_changed)
        screen.logicalDotsPerInchChanged.connect(self._on_metrics_changed)
        index = self.settings.value(self._settings_key(), 0, type=int)
        blocker = QtCore.QSignalBlocker(self.ui.comboBox_UiScale)
        self.ui.comboBox_UiScale.setCurrentIndex(index)
        del blocker
        self._apply_display()
        self._fit_timer.start(0)

    def _on_metrics_changed(self, *_):
        self._apply_display()
        self._fit_timer.start(0)

    def _on_scale_changed(self, index):
        if self._screen is None:
            return
        self.settings.setValue(self._settings_key(), index)
        self._apply_display()
        self._fit_timer.start(0)

    def _apply_scale(self, scale):
        for widget, base_font in self._fonts:
            font = QtGui.QFont(base_font)
            if base_font.pointSizeF() > 0:
                font.setPointSizeF(base_font.pointSizeF() * scale)
            else:
                font.setPixelSize(max(1, round(base_font.pixelSize() * scale)))
            widget.setFont(font)
        self.ui.classicCanvas.setMinimumSize(
            round(self._canvas_size.width() * scale),
            round(self._canvas_size.height() * scale))
        for widget, rect in self._fixed_rects:
            widget.setGeometry(round(rect.x() * scale), round(rect.y() * scale),
                               round(rect.width() * scale), round(rect.height() * scale))

    def _classic_window_size(self, available, scale, automatic):
        # 保留少量桌面边距；窗口尺寸不包含系统标题栏和边框。
        decoration = self.window.frameGeometry().size() - self.window.size()
        limit = available.size() - decoration - QtCore.QSize(16, 16)
        for attempt in range(4):
            self.window.layout().activate()
            self.ui.centralwidget.layout().activate()
            # maximumViewportSize 排除滚动条，避免已有滚动条抬高目标尺寸。
            chrome = self.window.size() - self.ui.scrollArea_Classic.maximumViewportSize()
            canvas = self.ui.classicCanvas.minimumSize()
            room = limit - chrome
            fitted_scale = min(scale, max(0.85, scale * min(
                room.width() / canvas.width(), room.height() / canvas.height())))
            if not automatic or fitted_scale >= scale - 0.001 or attempt == 3:
                break
            scale = fitted_scale
            self._apply_scale(scale)
        # 自动模式优先完整显示，达到可读性下限或用户选择大字时保留滚动条。
        desired = self.ui.classicCanvas.minimumSize() + chrome
        if desired.height() > limit.height():
            # 纵向滚动条占用宽度，预留后避免同时出现不必要的横向滚动。
            desired.setWidth(desired.width() + self.ui.scrollArea_Classic.style().pixelMetric(
                QtWidgets.QStyle.PM_ScrollBarExtent, None, self.ui.scrollArea_Classic))
        return desired.boundedTo(limit)

    def _apply_display(self):
        if self._updating or self._screen is None:
            return
        self._updating = True
        try:
            available = self._screen.availableGeometry()
            index = self.ui.comboBox_UiScale.currentIndex()
            scale = automatic_scale(available.size()) * self.SCALE_MULTIPLIERS[index]
            self._apply_scale(scale)
            normal_fit = not (self.window.isMaximized() or self.window.isFullScreen())
            if normal_fit:
                classic_size = self._classic_window_size(available, scale, index == 0)
            self.ui.comboBox_UiScale.setToolTip(
                f'{self._screen.name()} · 可用区域 {available.width()} × '
                f'{available.height()} 逻辑像素\n'
                f'当前基准字号 {self.window.font().pointSizeF():.1f} pt；'
                '百分比相对自动适配大小，按显示器记忆。')
            if normal_fit:
                self.window.resize(classic_size)
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

    def _clear_force_values(self):
        # 切换数据源后等待新样本，避免旧数值短暂显示在新单位下。
        for i in range(1, 7):
            getattr(self.ui, f'lineEdit_Fex{i}').clear()
