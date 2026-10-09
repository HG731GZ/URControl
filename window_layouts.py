"""从两份 Designer 文件切换布局，复用业务控件，保留信号、输入和视频。"""

from PyQt5 import QtCore, QtWidgets

from ui_main_window import Ui_MainWindow as ModernUi
from ui_classic_window import Ui_MainWindow as ClassicUi


def saved_layout(settings):
    mode = settings.value('layoutMode', 'modern')
    return mode if mode in ('modern', 'classic') else 'modern'


def _publish_widgets(window, ui, previous=None):
    # MainWindow 的业务代码仍通过 self.lineEdit_... 访问同一批控件。
    if previous is not None:
        for name, value in vars(previous).items():
            if name not in vars(ui) and getattr(window, name, None) is value:
                delattr(window, name)
    vars(window).update(vars(ui))


def create_layout(window, mode):
    ui = ClassicUi() if mode == 'classic' else ModernUi()
    ui.setupUi(window)
    _publish_widgets(window, ui)
    return ui


def _is_live_widget(name, widget):
    return (isinstance(widget, (QtWidgets.QAbstractButton, QtWidgets.QLineEdit,
                               QtWidgets.QPlainTextEdit, QtWidgets.QSlider))
            or name in ('label_IP_now', 'label_SpeedSlider', 'label_camera1',
                        'label_camera2', 'widget_RobotView'))


def switch_layout(window, previous, mode):
    """仅替换容器；不要重建 MainWindow、设备、计时器或控制信号。"""
    shell = QtWidgets.QMainWindow()
    ui = ClassicUi() if mode == 'classic' else ModernUi()
    ui.setupUi(shell)
    shell.ensurePolished()

    # 只复制 Designer 中的外观属性，enabled、readOnly、checked、value、
    # 输入文字、日志、正在采集的按钮文字等业务状态全部留在原控件上。
    appearance = ('minimumSize', 'maximumSize', 'sizePolicy', 'font', 'styleSheet',
                  'alignment', 'frameShape', 'scaledContents', 'tickPosition',
                  'cursor', 'autoDefault', 'default', 'placeholderText', 'toolTip')
    replacements = []
    for name, old in vars(previous).items():
        new = getattr(ui, name, None)
        if new is None or not _is_live_widget(name, old):
            continue
        properties = {key: new.property(key) for key in appearance
                      if new.metaObject().indexOfProperty(key) >= 0}
        replacements.append((name, old, new, properties, QtCore.QRect(new.geometry())))

    central = shell.takeCentralWidget()
    central.setFont(shell.font())
    # 先接入同一个顶层窗口，再移动控件，避免跨顶层窗口重建 OpenGL 上下文。
    central.setParent(window)
    central.hide()
    for name, old, new, properties, geometry in replacements:
        parent = new.parentWidget()
        old.setParent(parent)
        if parent.layout() is not None:
            item = parent.layout().replaceWidget(new, old)
            del item
        for key, value in properties.items():
            old.setProperty(key, value)
        # viewport、滚动条和 3D 子控件也回到模板的基准字号，防止每次
        # 切换时把上一轮已经缩放的字体再缩放一次。
        for child in old.findChildren(QtWidgets.QWidget):
            child.setFont(properties['font'])
        old.setGeometry(geometry)
        if isinstance(old, QtWidgets.QAbstractButton) and name != 'pushButton_Collect':
            old.setText(new.text())
            old.setToolTip(new.toolTip())
        setattr(ui, name, old)
        old.show()
        new.hide()
        new.setParent(None)
        new.deleteLater()
    ui.verticalLayout_RobotViewport = ui.widget_RobotView.layout()

    old_central = window.takeCentralWidget()
    old_central.hide()
    old_central.deleteLater()
    old_status = previous.statusbar.currentMessage()
    for name in ('menubar', 'statusbar'):
        old = getattr(previous, name)
        old.hide()
        old.setParent(None)
        old.deleteLater()
    for value in vars(previous).values():
        if isinstance(value, QtWidgets.QAction):
            value.deleteLater()
    # .ui 中的 QAction 由临时窗口持有，随菜单一并移交到主窗口。
    for value in vars(ui).values():
        if isinstance(value, QtWidgets.QAction):
            value.setParent(window)
    window.setMenuBar(ui.menubar)
    window.setStatusBar(ui.statusbar)
    ui.statusbar.showMessage(old_status)
    window.setFont(shell.font())
    window.setStyleSheet(shell.styleSheet())
    window.setMinimumSize(shell.minimumSize())
    window.setWindowTitle(shell.windowTitle())
    window.setCentralWidget(central)
    # 切换界面时使用该 .ui 的初始大小，显示控制器再适配当前屏幕。
    if window.isMaximized() or window.isFullScreen():
        window.showNormal()
    window.resize(shell.size())
    _publish_widgets(window, ui, previous)
    central.show()
    shell.deleteLater()
    return ui
