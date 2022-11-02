from python_qt_binding import QtWidgets

from autoware_adapi_v1_msgs.msg import OperationModeState
from tier4_system_msgs.srv import ChangeOperationMode
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile

mode_text = {
    OperationModeState.UNKNOWN: "UNKNOWN",
    OperationModeState.STOP: "STOP",
    OperationModeState.AUTONOMOUS: "AUTONOMOUS",
    OperationModeState.LOCAL: "LOCAL",
    OperationModeState.REMOTE: "REMOTE",
}

class OperationModeApiWidget(QtWidgets.QWidget):

    def __init__(self, api):
        super().__init__()
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

        self.button_stop_mode.clicked.connect(lambda: self.set_operation_mode(ChangeOperationMode.Request.STOP))
        self.button_auto_mode.clicked.connect(lambda: self.set_operation_mode(ChangeOperationMode.Request.AUTONOMOUS))
        self.button_local_mode.clicked.connect(lambda: self.set_operation_mode(ChangeOperationMode.Request.LOCAL))
        self.button_remote_mode.clicked.connect(lambda: self.set_operation_mode(ChangeOperationMode.Request.REMOTE))

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

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        api.create_subscription(OperationModeState, "/system/operation_mode/state", self.on_state, qos)

        self.cli_set_mode = api.create_client(ChangeOperationMode, "/system/operation_mode/change_operation_mode")

    def set_operation_mode(self, mode):
        req = ChangeOperationMode.Request()
        req.mode = mode
        self.cli_set_mode.call_async(req)

    def on_state(self, msg):
        self.operation_mode.setText(mode_text[msg.mode])
        self.autoware_control.setText(str(msg.is_autoware_control_enabled))
        self.transition.setText(str(msg.is_in_transition))
        self.button_stop_mode.setProperty("status", "enable" if msg.is_stop_mode_available else "disable")
        self.button_auto_mode.setProperty("status", "enable" if msg.is_autonomous_mode_available else "disable")
        self.button_local_mode.setProperty("status", "enable" if msg.is_local_mode_available else "disable")
        self.button_remote_mode.setProperty("status", "enable" if msg.is_remote_mode_available else "disable")

        for button in (self.button_stop_mode, self.button_auto_mode, self.button_local_mode, self.button_remote_mode):
            button.style().unpolish(button)
            button.style().polish(button)
