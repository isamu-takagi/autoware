#!/usr/bin/env python3
import rclpy
import yaml

class WaitTimer:

    def __init__(self, node):
        self.node = node
        self.wait = True

    def on_timer(self):
        self.wait = False

    def spin(self):
        timer = self.node.create_timer(3.0, self.on_timer)
        while self.wait:
            rclpy.spin_once(self.node)
        self.node.destroy_timer(timer)


class SaveGraph:

    def __init__(self, node):
        self.node = node

    def print_graph_info(self):
        graph = dict()
        graph["nodes"] = self.collect_node_info(self.node)
        graph["links"] = self.collect_link_info(self.node)
        print(yaml.safe_dump(graph, default_flow_style=None))

    @classmethod
    def compress_name_and_types(cls, name_and_types):
        return [[name, *types] for name, types in name_and_types]

    @classmethod
    def collect_node_info(cls, node):
        nodes = list()
        for names in node.get_node_names_and_namespaces():
            info = dict()
            info["name"] = names
            info["subs"] = cls.compress_name_and_types(node.get_subscriber_names_and_types_by_node(*names))
            info["pubs"] = cls.compress_name_and_types(node.get_publisher_names_and_types_by_node(*names))
            info["clis"] = cls.compress_name_and_types(node.get_client_names_and_types_by_node(*names))
            info["srvs"] = cls.compress_name_and_types(node.get_service_names_and_types_by_node(*names))
            nodes.append(info)
        return nodes

    @classmethod
    def collect_link_info(cls, node):
        # node.get_topic_names_and_types()
        # node.get_subscriptions_info_by_topic(name)
        # node.get_publishers_info_by_topic(name)
        return cls.compress_name_and_types(node.get_topic_names_and_types())


def main():
    rclpy.init()
    node = rclpy.create_node(node_name="print", namespace="interfaces")
    wait = WaitTimer(node)
    wait.spin()
    save = SaveGraph(node)
    save.print_graph_info()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
