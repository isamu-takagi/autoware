#!/usr/bin/env python3
import collections
import os
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


class PortInfo:

    builtins = ["rcl_interfaces", "composition_interfaces"]

    def __init__(self, type, name, msgs):
        self.type = type
        self.name = name
        self.msgs = msgs

    def is_builtin_interface(self):
        for msg in self.msgs:
            for pkg in self.builtins:
                if msg.startswith(pkg + "/"):
                    return True
        return False

    def dump(self):
        return [self.type, self.name, *self.msgs]


class LinkInfo:

    def __init__(self):
        self.name = None
        self.msgs = set()
        self.uses = []

    def register_node(self, node, port):
        self.msgs |= set(port.msgs)
        self.uses.append([port.type, node.full_name])

    def dump(self):
        return {"name": self.name, "msgs": list(self.msgs), "uses": self.uses}


class NodeInfo:

    def __init__(self, node_name, namespace):
        self.node_name = node_name
        self.namespace = namespace
        self.interface = []

    @property
    def full_name(self):
        return os.path.join(self.namespace, self.node_name)

    def register_interface(self, type, name_and_msgs):
        interface = [PortInfo(type, name, msgs) for name, msgs in name_and_msgs]
        interface = [port for port in interface if not port.is_builtin_interface()]
        self.interface.extend(interface)

    def dump(self):
        return {"full_name": self.full_name, "interface": [port.dump() for port in self.interface]}


class SaveGraph:

    def __init__(self, node):
        self.node = node

    def print_graph_info(self):
        nodes = self.collect_node_info(self.node)
        links = self.collect_link_info(nodes)
        graph = dict()
        graph["nodes"] = [info.dump() for info in nodes]
        graph["links"] = [info.dump() for info in links]
        print(yaml.safe_dump(graph, default_flow_style=None, width=200))

    @classmethod
    def compress_name_and_types(cls, name_and_types):
        return [[name, *types] for name, types in name_and_types]

    @classmethod
    def collect_node_info(cls, node):
        nodes = list()
        for node_name, namespace in node.get_node_names_and_namespaces():
            info = NodeInfo(node_name, namespace)
            info.register_interface("sub", node.get_subscriber_names_and_types_by_node(info.node_name, info.namespace))
            info.register_interface("pub", node.get_publisher_names_and_types_by_node(info.node_name, info.namespace))
            info.register_interface("cli", node.get_client_names_and_types_by_node(info.node_name, info.namespace))
            info.register_interface("srv", node.get_service_names_and_types_by_node(info.node_name, info.namespace))
            nodes.append(info)
        return nodes

    @classmethod
    def collect_link_info(cls, nodes):
        links = collections.defaultdict(LinkInfo)
        for node in nodes:
            for port in node.interface:
                links[port.name].register_node(node, port)
        for name, link in links.items():
            link.name = name
        return list(links.values())


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

# node.get_topic_names_and_types()
# node.get_subscriptions_info_by_topic(name)
# node.get_publishers_info_by_topic(name)
