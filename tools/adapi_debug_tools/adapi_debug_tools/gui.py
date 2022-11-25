import rclpy
import rclpy.signals
import signal
import sys
import threading
from python_qt_binding import QtCore, QtWidgets
from adapi_debug_tools.api import Adapi
from adapi_debug_tools.widgets import AutowareStateWidget

class MainSettings(QtCore.QSettings):

    def restore(self, name, func):
        if self.contains(name):
            func(self.value(name))


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, adapi: Adapi):
        super().__init__()
        self.adapi = adapi
        self.widget = QtWidgets.QTabWidget()
        self.widget.addTab(AutowareStateWidget(adapi), "main")
        self.setWindowTitle("Autoware AD API")
        self.setCentralWidget(self.widget)
        self.load_config()

    def closeEvent(self, event):
        self.save_config()
        super(MainWindow, self).closeEvent(event)

    def load_config(self):
        settings = MainSettings("autoware", "adasmap_editor")
        settings.restore("MainWindow/geometry", self.restoreGeometry)
        self.adapi.settings.load()

    def save_config(self):
        settings = MainSettings("autoware", "adasmap_editor")
        settings.setValue("MainWindow/geometry", self.saveGeometry())
        self.adapi.settings.save()


class RosExecution:

    def __init__(self):
        rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        self.adapi = Adapi("adapi_debug_tools")
        self.thread = threading.Thread(target=rclpy.spin, args=[self.adapi.node])

    def start(self):
        self.thread.start()

    def shutdown(self):
        rclpy.shutdown()
        self.thread.join()


class AppExecution:

    def __init__(self, ros):
        self.application = QtWidgets.QApplication(sys.argv)
        self.window = MainWindow(ros.adapi)

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
