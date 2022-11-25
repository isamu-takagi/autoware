from python_qt_binding import QtCore, QtWidgets

from adapi_debug_tools.api import Adapi
from adapi_debug_tools.widgets.settings import PoseCovDialog
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosidl_runtime_py.set_message import set_message_fields


class LocalizationWidgets:

    def __init__(self, adapi: Adapi, parent: QtWidgets.QWidget):
        self.adapi = adapi
        self.parent = parent
        self.sub_state = adapi.localization.state.subscribe(self.on_state)
        self.client = adapi.localization.initialize
        self.label_state = QtWidgets.QLabel("unknown")
        self.label_state.setAlignment(QtCore.Qt.AlignCenter)
        self.label_state.setStyleSheet("QLabel { border: 1px solid; padding: 2px; }")
        self.button_gnss_execute = QtWidgets.QPushButton("init gnss")
        self.button_pose_execute = QtWidgets.QPushButton("init pose")
        self.button_pose_setting = QtWidgets.QPushButton("pose setting")
        self.button_gnss_execute.clicked.connect(self.on_gnss_request)
        self.button_pose_execute.clicked.connect(self.on_pose_request)
        self.button_pose_setting.clicked.connect(self.on_pose_setting)

        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addWidget(self.button_gnss_execute)
        button_layout.addWidget(self.button_pose_execute)
        button_layout.addWidget(self.button_pose_setting)

        self.layout = [
            ("localization state", self.label_state),
            ("localization", button_layout),
        ]

    def on_gnss_request(self):
        req = InitializeLocalization.Request()
        self.future = self.client.call_async(req)

    def on_pose_request(self):
        pose = PoseWithCovarianceStamped()
        set_message_fields(pose, self.adapi.settings.get_data("initial-pose"))
        req = InitializeLocalization.Request()
        req.pose = [pose]
        self.future = self.client.call_async(req)

    def on_pose_setting(self):
        dialog = PoseCovDialog(self.adapi, self.parent)
        dialog.exec_()

    def on_state(self, msg):
        state_text = {
            LocalizationInitializationState.UNKNOWN: "unknown",
            LocalizationInitializationState.UNINITIALIZED: "uninitialized",
            LocalizationInitializationState.INITIALIZING: "initializing",
            LocalizationInitializationState.INITIALIZED: "initialized",
        }
        self.label_state.setText(state_text[msg.state])
