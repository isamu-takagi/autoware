from functools import partial
from python_qt_binding import QtCore, QtWidgets

from adapi_debug_tools.api import Adapi
from autoware_adapi_version_msgs.srv import InterfaceVersion


class InterfaceWidgets:

    def __init__(self, adapi: Adapi):
        self.client = adapi.interface.version
        self.label = QtWidgets.QLabel("unknown")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setStyleSheet("QLabel { border: 1px solid; padding: 2px; }")
        self.button = QtWidgets.QPushButton("get")
        self.button.clicked.connect(self.on_version_request)

        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.label, 4)
        layout.addWidget(self.button, 1)

        self.layout = [
            ("interface version", layout),
        ]

    def on_version_request(self):
        self.future = self.client.call_async(InterfaceVersion.Request())
        self.future.add_done_callback(self.on_version_response)

    def on_version_response(self, future):
        response = future.result()
        self.label.setText(f"{response.major}.{response.minor}.{response.patch}")
