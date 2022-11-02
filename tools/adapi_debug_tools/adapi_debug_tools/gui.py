import rclpy
import rclpy.signals
import signal
import sys
import threading
from python_qt_binding import QtCore, QtWidgets
from adapi_debug_tools.api import ApiNode
from adapi_debug_tools.widgets import OperationModeApiWidget

class MainSettings(QtCore.QSettings):

    def restore(self, name, func):
        if self.contains(name):
            func(self.value(name))


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, api):
        super().__init__()
        self.setWindowTitle("Autoware AD API")
        self.setCentralWidget(OperationModeApiWidget(api))
        self.load_config()

    def closeEvent(self, event):
        self.save_config()
        super(MainWindow, self).closeEvent(event)

    def load_config(self):
        settings = MainSettings("autoware", "adasmap_editor")
        settings.restore("MainWindow/geometry", self.restoreGeometry)
        # self.centralWidget().load_config(settings)

    def save_config(self):
        settings = MainSettings("autoware", "adasmap_editor")
        settings.setValue("MainWindow/geometry", self.saveGeometry())
        # self.centralWidget().save_config(settings)


class RosExecution(object):

    def __init__(self):
        rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        self.node = ApiNode("adapi_debug_tools")
        self.thread = threading.Thread(target=rclpy.spin, args=[self.node])

    def start(self):
        self.thread.start()

    def shutdown(self):
        rclpy.shutdown()
        self.thread.join()


class AppExecution(object):

    def __init__(self, ros):
        self.application = QtWidgets.QApplication(sys.argv)
        self.window = MainWindow(ros.node)

    def start(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.window.show()
        self.result = self.application.exec_()

    def shutdown(self):
        sys.exit(self.result)


def main():

    ros = RosExecution()
    app = AppExecution(ros)
    ros.start()
    app.start()
    ros.shutdown()
    app.shutdown()
