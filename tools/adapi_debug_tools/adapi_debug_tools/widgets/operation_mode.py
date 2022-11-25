from functools import partial
from python_qt_binding import QtCore, QtWidgets

from adapi_debug_tools.api import Adapi
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode


class OperationModeWidgets:

    def __init__(self, adapi: Adapi):
        adapi.operation_mode.create()
        self.sub_state = adapi.operation_mode.state.subscribe(self.on_state)
        self.cli_mode = adapi.operation_mode.change_mode

        self.buttons = {}
        self.mode_label = QtWidgets.QLabel("unknown")
        self.mode_label.setAlignment(QtCore.Qt.AlignCenter)
        self.mode_label.setStyleSheet("QLabel { border: 1px solid; padding: 2px; }")

        button_settings = [
            ("enable", adapi.operation_mode.enable_control),
            ("disable", adapi.operation_mode.disable_control),
            ("stop", adapi.operation_mode.change_stop),
            ("auto", adapi.operation_mode.change_auto),
            ("local", adapi.operation_mode.change_local),
            ("remote", adapi.operation_mode.change_remote),
        ]

        button_style = 'QPushButton[status="enable"] { background-color: lime; }'
        for name, api in button_settings:
            button = QtWidgets.QPushButton(name)
            button.setStyleSheet(button_style)
            button.clicked.connect(partial(self.change_mode, api))
            self.buttons[name] = button

        mode_layout = QtWidgets.QHBoxLayout()
        for name in ("stop", "auto", "local", "remote"):
            mode_layout.addWidget(self.buttons[name])

        ctrl_layout = QtWidgets.QHBoxLayout()
        for name in ("enable", "disable"):
            ctrl_layout.addWidget(self.buttons[name])

        self.layout = [
            ("operation mode state", self.mode_label),
            ("change operation mode", mode_layout),
            ("change autoware control", ctrl_layout),
        ]

    def change_mode(self, client):
        client.call_async(ChangeOperationMode.Request())

    def on_state(self, msg):
        change_availables = {
            "stop": msg .is_stop_mode_available,
            "auto": msg.is_autonomous_mode_available,
            "local": msg.is_local_mode_available,
            "remote": msg.is_remote_mode_available,
        }
        for name in ("stop", "auto", "local", "remote"):
            button = self.buttons[name]
            status = "enable" if change_availables[name] else "disable"
            if button.property("status") != status:
                button.setProperty("status", status)
                button.style().polish(button)

        mode_text = {
            OperationModeState.UNKNOWN: "unknown",
            OperationModeState.STOP: "stop",
            OperationModeState.AUTONOMOUS: "auto",
            OperationModeState.LOCAL: "local",
            OperationModeState.REMOTE: "remote",
        }
        text1 = "transition to " if msg.is_in_transition else ""
        text2 = mode_text[msg.mode]
        text3 = "autoware" if msg.is_autoware_control_enabled else "manual"
        self.mode_label.setText(f"{text1} {text2} ({text3} control)")
