import rclpy.node


class ApiNode(rclpy.node.Node):

    def __init__(self, node_name):
        super().__init__(node_name)
