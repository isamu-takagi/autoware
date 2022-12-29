#!/usr/bin/env python3
import argparse
import yaml
from collections import defaultdict
from pathlib import Path

class Link:

    def __init__(self, data):
        self.name = data[0]
        self.types = data[1:]

    @staticmethod
    def is_target_link(link):
        ignore = frozenset(["rcl_interfaces", "composition_interfaces", "tf2_msgs"])
        for type in link.types:
            package = type.split("/")[0]
            if package in ignore:
                return False
        return True

    @staticmethod
    def filter(links):
        return list(filter(Link.is_target_link, links))


class Node:

    def __init__(self, data):
        self.namespace = data["name"][1]
        self.node_name = data["name"][0]
        self.full_name = self.namespace + "/" + self.node_name
        self.component = self.__component(self.namespace, self.node_name)
        self.pubs = Link.filter(Link(data) for data in data["pubs"])
        self.subs = Link.filter(Link(data) for data in data["subs"])
        self.clis = Link.filter(Link(data) for data in data["clis"])
        self.srvs = Link.filter(Link(data) for data in data["srvs"])

    @property
    def links(self):
        yield ("pub", self.pubs)
        yield ("sub", self.subs)
        yield ("cli", self.clis)
        yield ("srv", self.srvs)

    @staticmethod
    def __component(namespace, node_name):
        parts = namespace.split("/")
        return parts[1] if parts[1] else node_name

    @staticmethod
    def is_target_node(node):
        return node.node_name != "rviz2"

    @staticmethod
    def filter(nodes):
        return list(filter(Node.is_target_node, nodes))

class Graph:

    def __init__(self, path: Path):
        graph = yaml.safe_load(path.read_text())
        self.nodes = Node.filter(Node(data) for data in graph["nodes"])


class Interface:

    def __init__(self):
        self.components = set()
        self.full_names = set()

    def insert(self, node: Node):
        self.components.add(node.component)
        self.full_names.add(node.full_name)


def exec_dump(args):
    graph = yaml.safe_load(Path(__file__).parent.joinpath("psim.yaml").read_text())
    nodes = Node.filter(Node(data) for data in graph["nodes"])
    msgs = defaultdict(Interface)
    srvs = defaultdict(Interface)

    for node in nodes:
        for pub in node.pubs: msgs[pub.name].insert(node)
        for sub in node.subs: msgs[sub.name].insert(node)
        for cli in node.clis: srvs[cli.name].insert(node)
        for srv in node.srvs: srvs[srv.name].insert(node)

    for name, data in msgs.items():
        if 2 <= len(data.components):
            modules = ", ".join(data.components)
            print(f"{name}\t{modules}")

    for name, data in srvs.items():
        if 2 <= len(data.components):
            modules = ", ".join(data.components)
            print(f"{name}\t{modules}")


def exec_find(args):
    graph = Graph(Path(__file__).parent.joinpath("psim.yaml"))
    for node in graph.nodes:
        for type, links in node.links:
            if any(link.name == args.name for link in links):
                print(type, node.node_name)


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command")
    subparsers.required = True

    dump_parser = subparsers.add_parser("dump")
    dump_parser.set_defaults(func=exec_dump)

    dump_parser = subparsers.add_parser("find")
    dump_parser.set_defaults(func=exec_find)
    dump_parser.add_argument("name")

    args = parser.parse_args()
    args.func(args)

if __name__ == "__main__":
    main()
