#!/usr/bin/env python3
import rclpy
import yaml
from rclpy.node import Node

def compress_name_and_types(name_and_types):
    return [[name, *types] for name, types in name_and_types]

def collect_node_info(node: Node):
    nodes = list()
    for names in node.get_node_names_and_namespaces():
        info = dict()
        info["name"] = names
        info["subs"] = compress_name_and_types(node.get_subscriber_names_and_types_by_node(*names))
        info["pubs"] = compress_name_and_types(node.get_publisher_names_and_types_by_node(*names))
        info["clis"] = compress_name_and_types(node.get_client_names_and_types_by_node(*names))
        info["srvs"] = compress_name_and_types(node.get_service_names_and_types_by_node(*names))
        nodes.append(info)
    return nodes


def collect_link_info(node: Node):
    # node.get_topic_names_and_types()
    # node.get_subscriptions_info_by_topic(name)
    # node.get_publishers_info_by_topic(name)
    return compress_name_and_types(node.get_topic_names_and_types())


def print_graph_info(node: Node):
    graph = dict()
    graph["nodes"] = collect_node_info(node)
    graph["links"] = collect_link_info(node)
    print(yaml.safe_dump(graph, default_flow_style=None))


if __name__ == "__main__":
    rclpy.init()
    node = Node("interfaces")
    print_graph_info(node)
    rclpy.shutdown()
