from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode


class Adapi:

    def __init__(self, node_name):
        self.node = Node(node_name)
        self.operation_mode = AdapiOperationMode(self.node)


class AdapiOperationMode:

    def __init__(self, node):
        self.__node = node
        self.state = None
        self.change_mode = None

    def create(self):
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state = AdapiSubscription(self.__node, OperationModeState, "/api/operation_mode/state", qos)
        self.enable_control = AdapiClient(self.__node, ChangeOperationMode, "/api/operation_mode/enable_autoware_control")
        self.disable_control = AdapiClient(self.__node, ChangeOperationMode, "/api/operation_mode/disable_autoware_control")
        self.change_stop = AdapiClient(self.__node, ChangeOperationMode, "/api/operation_mode/change_to_stop")
        self.change_auto = AdapiClient(self.__node, ChangeOperationMode, "/api/operation_mode/change_to_autonomous")
        self.change_local = AdapiClient(self.__node, ChangeOperationMode, "/api/operation_mode/change_to_local")
        self.change_remote = AdapiClient(self.__node, ChangeOperationMode, "/api/operation_mode/change_to_remote")


class AdapiSubscription:

    def __init__(self, node: Node, type: type, name: str, qos: QoSProfile):
        self.__interface = node.create_subscription(type, name, self.__on_message, qos)
        self.__callbacks = []

    def __on_message(self, msg):
        for callback in self.__callbacks: callback(msg)

    def subscribe(self, callback):
        self.__callbacks.append(callback)


class AdapiClient:

    def __init__(self, node: Node, type: type, name: str):
        self.__interface = node.create_client(type, name)

    def call_async(self, *args, **kwargs):
        self.__interface.call_async(*args, **kwargs)
