"""离线检查显示器切换、布局可达性和相机比例，不导入机器人或相机驱动。"""

import os
import tempfile
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtGui, QtWidgets

from ui_main_window import Ui_MainWindow
from ui_widgets import AspectRatioLabel
from window_display import WindowDisplayController, configure_high_dpi
from window_layouts import switch_layout


class ScreenStub(QtCore.QObject):
    availableGeometryChanged = QtCore.pyqtSignal(QtCore.QRect)
    logicalDotsPerInchChanged = QtCore.pyqtSignal(float)

    def __init__(self, name, width, height, x=0):
        super().__init__()
        self._name = name
        self.area = QtCore.QRect(x, 0, width, height)

    def availableGeometry(self):
        return QtCore.QRect(self.area)

    def name(self):
        return self._name

    def manufacturer(self):
        return 'test'

    def model(self):
        return self._name

    def serialNumber(self):
        return self._name


class DisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        configure_high_dpi()
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.settings = QtCore.QSettings(
            os.path.join(self.tmp.name, 'display.ini'), QtCore.QSettings.IniFormat)
        self.window = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.window)
        self.display = WindowDisplayController(self.window, self.ui, self.settings)
        self.four_k = ScreenStub('4K-200', 1920, 1040)
        self.two_k = ScreenStub('2K-100', 2560, 1400, x=1920)
        # offscreen 的宿主屏幕通常仅 800×600；从一个真实目标尺寸开始。
        self.display._available = None
        self.display._on_screen_changed(self.four_k)
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.tmp.cleanup()

    def test_screen_switch_preserves_input_and_remembers_each_display(self):
        self.ui.lineEdit_QT1.setText('23.456')
        four_k_font = self.window.font().pointSizeF()
        self.ui.comboBox_UiScale.setCurrentIndex(1)
        self.display._on_screen_changed(self.two_k)
        self.app.processEvents()
        self.assertEqual(self.ui.comboBox_UiScale.currentIndex(), 0)
        self.assertGreater(self.window.font().pointSizeF(), four_k_font * 1.2)
        for widget in (self.ui.lineEdit_QA1, self.ui.pushButton_ConnectUR,
                       self.ui.label_Subtitle, self.ui.tabWidget_Control.tabBar()):
            self.assertAlmostEqual(widget.font().pointSizeF(), self.window.font().pointSizeF())
        self.assertTrue(self.two_k.area.contains(self.window.frameGeometry()))
        self.ui.comboBox_UiScale.setCurrentIndex(3)
        self.display._on_screen_changed(self.four_k)
        self.app.processEvents()
        self.assertEqual(self.ui.comboBox_UiScale.currentIndex(), 1)
        self.assertAlmostEqual(self.window.font().pointSizeF(), four_k_font * 0.9)
        self.assertEqual(self.ui.lineEdit_QT1.text(), '23.456')
        self.assertTrue(self.four_k.area.contains(self.window.frameGeometry()))

    def test_layout_at_both_screen_sizes_and_larger_text(self):
        for screen in (self.four_k, self.two_k):
            self.display._on_screen_changed(screen)
            for scale_index in (0, 3):
                self.ui.comboBox_UiScale.setCurrentIndex(scale_index)
                for control in range(self.ui.tabWidget_Control.count()):
                    self.ui.tabWidget_Control.setCurrentIndex(control)
                    for telemetry in range(self.ui.tabWidget_Telemetry.count()):
                        with self.subTest(screen=screen.name(), scale=scale_index,
                                          control=control, telemetry=telemetry):
                            self.ui.tabWidget_Telemetry.setCurrentIndex(telemetry)
                            self.app.processEvents()
                            self.assertEqual(self.ui.scrollArea_Control.horizontalScrollBar().maximum(), 0)
                            for button in (self.ui.pushButton_Stop, self.ui.pushButton_Shutdown,
                                           self.ui.pushButton_Collect):
                                rect = QtCore.QRect(button.mapTo(self.window, QtCore.QPoint()), button.size())
                                self.assertTrue(self.window.rect().contains(rect))
                                self.assertGreaterEqual(button.width(), button.fontMetrics().horizontalAdvance(button.text()) + 16)
                            self.assertGreater(self.ui.widget_RobotView.height(), 180)
                            self.assertGreater(self.ui.label_camera1.width(), 140)

    def test_work_area_change_updates_font_without_resetting_state(self):
        self.ui.tabWidget_Control.setCurrentIndex(1)
        old_font = self.window.font().pointSizeF()
        self.four_k.area = QtCore.QRect(0, 0, 1600, 960)
        self.four_k.availableGeometryChanged.emit(self.four_k.area)
        self.app.processEvents()
        self.assertLess(self.window.font().pointSizeF(), old_font)
        self.assertEqual(self.ui.tabWidget_Control.currentIndex(), 1)
        self.assertTrue(self.four_k.area.contains(self.window.frameGeometry()))

    def test_default_window_leaves_desktop_space(self):
        for screen in (self.four_k, self.two_k):
            self.display._on_screen_changed(screen)
            self.app.processEvents()
            self.assertFalse(self.window.isMaximized())
            self.assertFalse(self.window.isFullScreen())
            self.assertLess(self.window.width(), screen.area.width() * 0.8)
            self.assertLess(self.window.height(), screen.area.height() * 0.84)

    def test_layout_switch_preserves_live_widgets_signals_and_state(self):
        target = self.ui.lineEdit_QT1
        collect = self.ui.pushButton_Collect
        target.setText('23.456')
        target.setReadOnly(True)
        self.ui.checkBox_FTSwitch.setChecked(True)
        self.ui.lineEdit_Fex1.setText('7.89')
        collect.setText('采集结束')
        clicks = []
        collect.clicked.connect(lambda: clicks.append(True))
        viewport = QtWidgets.QWidget()
        self.ui.verticalLayout_RobotViewport.addWidget(viewport)
        for mode in ('classic', 'modern', 'classic', 'modern'):
            with self.subTest(mode=mode):
                self.display.dispose()
                self.ui = switch_layout(self.window, self.ui, mode)
                self.display = WindowDisplayController(self.window, self.ui, self.settings)
                self.display._available = None
                self.display._on_screen_changed(self.four_k)
                self.app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
                self.app.processEvents()
                self.assertIs(self.ui.lineEdit_QT1, target)
                self.assertTrue(target.isReadOnly())
                self.assertEqual(target.text(), '23.456')
                self.assertTrue(self.ui.checkBox_FTSwitch.isChecked())
                self.assertEqual(self.ui.lineEdit_Fex1.text(), '7.89')
                self.assertIs(self.ui.pushButton_Collect, collect)
                self.assertEqual(collect.text(), '采集结束')
                self.assertIs(viewport.window(), self.window)
                self.assertIs(self.ui.verticalLayout_RobotViewport.itemAt(0).widget(), viewport)
                self.assertEqual(collect.receivers(collect.clicked), 1)
                self.assertEqual(clicks, [])
                # 销毁旧布局后再改字号，可发现仍引用旧控件/旧显示控制器的问题。
                self.ui.comboBox_UiScale.setCurrentIndex(2)
                self.app.processEvents()
                log = self.ui.plainTextEdit_DashboardMessage
                self.assertAlmostEqual(log.viewport().font().pointSizeF(), log.font().pointSizeF())
                self.assertAlmostEqual(viewport.font().pointSizeF(),
                                       self.ui.widget_RobotView.font().pointSizeF())
        collect.click()
        self.assertEqual(clicks, [True])

    def test_axis_colors_and_disabled_controls_have_distinct_fill(self):
        self.ui.tabWidget_Control.setCurrentIndex(1)
        for index, rgb in enumerate(((246, 97, 81), (143, 240, 164), (153, 193, 241),
                                      (165, 29, 45), (38, 162, 105), (26, 95, 180)), 1):
            for prefix in ('T', 'TW'):
                button = getattr(self.ui, f'pushButton_{prefix}Up{index}')
                button.setEnabled(True)
                self.app.processEvents()
                image = button.grab().toImage()
                ratio = image.devicePixelRatio()
                self.assertEqual(image.pixelColor(round(7 * ratio), round(7 * ratio)).getRgb()[:3], rgb)
                button.setEnabled(False)
                self.app.processEvents()
                image = button.grab().toImage()
                self.assertEqual(image.pixelColor(round(7 * ratio), round(7 * ratio)).getRgb()[:3], (216, 223, 232))

    def test_force_labels_match_selected_data_source(self):
        self.assertEqual(self.ui.label_ForceAxis1.text(), 'Fx · N')
        self.ui.lineEdit_Fex1.setText('12.345')
        self.ui.checkBox_FTSwitch.setChecked(True)
        self.assertEqual(self.ui.label_ForceAxis1.text(), 'J1 · N·m')
        self.assertEqual(self.ui.lineEdit_Fex1.text(), '')
        self.assertEqual(self.ui.label_ForceAxis6.text(), 'J6 · N·m')
        self.ui.checkBox_FTSwitch.setChecked(False)
        self.assertEqual(self.ui.label_ForceAxis6.text(), 'Mz · N·m')

    def test_hidden_log_announces_new_messages_without_switching_controls(self):
        tabs = self.ui.tabWidget_Telemetry
        index = tabs.indexOf(self.ui.tab_Log)
        self.ui.plainTextEdit_DashboardMessage.appendPlainText('机器人连接失败')
        self.assertEqual(tabs.currentIndex(), 0)
        self.assertEqual(tabs.tabText(index), '运行日志 (1)')
        tabs.setCurrentIndex(index)
        self.assertEqual(tabs.tabText(index), '运行日志')

    def test_camera_frames_keep_aspect_ratio_after_resize_and_clear(self):
        label = AspectRatioLabel()
        label.setStyleSheet('background: black; color: white;')
        label.show()
        for frame_size, widget_size in (((160, 90), (300, 240)),
                                        ((90, 160), (300, 240)),
                                        ((160, 90), (220, 320))):
            with self.subTest(frame=frame_size, widget=widget_size):
                frame = QtGui.QPixmap(*frame_size)
                frame.fill(QtCore.Qt.red)
                label.setPixmap(frame)
                label.resize(*widget_size)
                self.app.processEvents()
                image = label.grab().toImage()
                cx, cy = image.width() // 2, image.height() // 2
                xs = [x for x in range(image.width()) if image.pixelColor(x, cy).red() > 240]
                ys = [y for y in range(image.height()) if image.pixelColor(cx, y).red() > 240]
                self.assertAlmostEqual(len(xs) / len(ys), frame_size[0] / frame_size[1], delta=0.025)
                self.assertEqual(label.pixmap().size(), frame.size())
        label.setText('相机未连接')
        self.assertTrue(label.pixmap().isNull())
        self.assertEqual(label.text(), '相机未连接')
        label.close()


if __name__ == '__main__':
    unittest.main()
