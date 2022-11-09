from python_qt_binding import QtWidgets

from adapi_debug_tools.api import Adapi
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode

mode_text = {
    OperationModeState.UNKNOWN: "UNKNOWN",
    OperationModeState.STOP: "STOP",
    OperationModeState.AUTONOMOUS: "AUTONOMOUS",
    OperationModeState.LOCAL: "LOCAL",
    OperationModeState.REMOTE: "REMOTE",
}

class OperationModeApiWidget(QtWidgets.QWidget):

    def __init__(self, adapi: Adapi):
        super().__init__()
        adapi.operation_mode.create()
        self.sub_state = adapi.operation_mode.state.subscribe(self.on_state)
        self.cli_mode = adapi.operation_mode.change_mode

        self.operation_mode = QtWidgets.QLabel("UNKNOWN")
        self.autoware_control = QtWidgets.QLabel("UNKNOWN")
        self.transition = QtWidgets.QLabel("UNKNOWN")
        self.button_stop_mode = QtWidgets.QPushButton("STOP")
        self.button_auto_mode = QtWidgets.QPushButton("AUTO")
        self.button_local_mode = QtWidgets.QPushButton("LOCAL")
        self.button_remote_mode = QtWidgets.QPushButton("REMOTE")
        self.button_enable = QtWidgets.QPushButton("enable")
        self.button_disable = QtWidgets.QPushButton("disable")

        style = '* [status="enable"] { background-color: lime; }'
        self.button_stop_mode.setStyleSheet(style)
        self.button_auto_mode.setStyleSheet(style)
        self.button_local_mode.setStyleSheet(style)
        self.button_remote_mode.setStyleSheet(style)
        self.button_enable.setStyleSheet(style)
        self.button_disable.setStyleSheet(style)

        self.button_stop_mode.clicked.connect(lambda: self.change_mode(adapi.operation_mode.change_stop))
        self.button_auto_mode.clicked.connect(lambda: self.change_mode(adapi.operation_mode.change_auto))
        self.button_local_mode.clicked.connect(lambda: self.change_mode(adapi.operation_mode.change_local))
        self.button_remote_mode.clicked.connect(lambda: self.change_mode(adapi.operation_mode.change_remote))
        self.button_enable.clicked.connect(lambda: self.change_mode(adapi.operation_mode.enable_control))
        self.button_disable.clicked.connect(lambda: self.change_mode(adapi.operation_mode.disable_control))

        mode_layout = QtWidgets.QHBoxLayout()
        mode_layout.addWidget(self.button_stop_mode)
        mode_layout.addWidget(self.button_auto_mode)
        mode_layout.addWidget(self.button_local_mode)
        mode_layout.addWidget(self.button_remote_mode)

        control_layout = QtWidgets.QHBoxLayout()
        control_layout.addWidget(self.button_enable)
        control_layout.addWidget(self.button_disable)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel("operation mode"), 0, 0)
        layout.addWidget(QtWidgets.QLabel("autoware control"), 1, 0)
        layout.addWidget(QtWidgets.QLabel("transition"), 2, 0)
        layout.addWidget(self.operation_mode, 0, 1)
        layout.addWidget(self.autoware_control, 1, 1)
        layout.addWidget(self.transition, 2, 1)
        layout.addLayout(mode_layout, 0, 2)
        layout.addLayout(control_layout, 1, 2)
        layout.setRowStretch(3, 1)
        self.setLayout(layout)


    def change_mode(self, client):
        client.call_async(ChangeOperationMode.Request())

    def on_state(self, msg):
        self.operation_mode.setText(mode_text[msg.mode])
        self.autoware_control.setText(str(msg.is_autoware_control_enabled))
        self.transition.setText(str(msg.is_in_transition))
        self.button_stop_mode.setProperty("status", "enable" if msg.is_stop_mode_available else "disable")
        self.button_auto_mode.setProperty("status", "enable" if msg.is_autonomous_mode_available else "disable")
        self.button_local_mode.setProperty("status", "enable" if msg.is_local_mode_available else "disable")
        self.button_remote_mode.setProperty("status", "enable" if msg.is_remote_mode_available else "disable")

        #for button in (self.button_stop_mode, self.button_auto_mode, self.button_local_mode, self.button_remote_mode):
        #    button.style().unpolish(button)
        #    button.style().polish(button)
