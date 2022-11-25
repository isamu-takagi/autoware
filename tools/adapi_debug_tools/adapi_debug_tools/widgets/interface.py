from python_qt_binding import QtCore, QtWidgets

from adapi_debug_tools.api import Adapi
from autoware_adapi_version_msgs.srv import InterfaceVersion


class InterfaceWidgets:

    def __init__(self, adapi: Adapi):
        self.client = adapi.interface.version
        self.button = QtWidgets.QPushButton("version")
        self.button.clicked.connect(self.on_version_request)

        self.layout = [
            ("interface version", self.button),
        ]

    def on_version_request(self):
        self.future = self.client.call_async(InterfaceVersion.Request())
        self.future.add_done_callback(self.on_version_response)

    def on_version_response(self, future):
        response = future.result()
        self.button.setText(f"{response.major}.{response.minor}.{response.patch}")
