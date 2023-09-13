#!/usr/bin/env python3
from argparse import ArgumentParser
from pathlib import Path
import yaml

autoware_components = frozenset([
    "map",
    "localization",
    "sensing",
    "perception",
    "planning",
    "control",
    "vehicle",
    "system",
    "simulation",
    "adapi",
    "tier4_api",
    "rviz",
])

spreadsheet = [
    "map",
    "localization",
    "sensing",
    "perception",
    "planning",
    "control",
    "vehicle",
    "system",
    "api",
    "rviz",
]

mapping = {
    "pose_initializer_node": "localization",
    "robot_state_publisher": "sensing",
    "pointcloud_container": "sensing",
    "occupancy_grid_map": "perception",
    "aggregator_node": "system",
    "static_map_to_odom_tf_publisher": "simulation",
    "default_ad_api": "adapi",
    "autoware_api": "tier4_api",
    "rviz2": "rviz",
    "rosbag2_player": "rosbag",
}

def classify_component(name, ifname):
    component = name.split("/")[1]
    component = mapping.get(component, component)
    if component == "rosbag":
        component = ifname.split("/")[1]
    if component not in autoware_components:
        raise RuntimeError("unknown component: " + component)
    return component

def remove_rviz(nodes):
    for node_type, node_name in nodes:
        if node_name != "/rviz2":
            yield node_type, node_name

def remove_transform_listener(nodes):
    for node_type, node_name in nodes:
        if not node_name.startswith("/transform_listener_impl_"):
            yield node_type, node_name

def remove_topic_state_monitor(nodes):
    for node_type, node_name in nodes:
        if not node_name.split("/")[-1].startswith("topic_state_monitor_"):
            yield node_type, node_name

class InterfaceInfo:

    def __init__(self, name):
        self.name = name
        self.types = set()
        self.nodes = set()
        self.components = set()

    def update(self, types, nodes):
        self.types.update(types)
        self.nodes.update(nodes)

    def classify(self):
        self.nodes = [InterfaceNode(self.name, *node) for node in self.nodes]
        self.components = {node.component for node in self.nodes}

class InterfaceNode:

    def __init__(self, ifname, type, name):
        self.type = type
        self.name = name
        self.component = classify_component(name, ifname)

def main():
    parser = ArgumentParser()
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args()

    interfaces = {}
    for path in args.paths:
        links = yaml.safe_load(Path(path).read_text())["links"]
        for link in links:
            name = link["name"]
            msgs = link["msgs"]
            uses = link["uses"]
            uses = remove_transform_listener(uses)
            uses = remove_topic_state_monitor(uses)
            interfaces.setdefault(name, InterfaceInfo(name))
            interfaces[name].update(msgs, uses)

    interfaces = list(interfaces.values())
    interfaces = [i for i in interfaces if not i.name.startswith("/events")]
    interfaces = [i for i in interfaces if not i.name.startswith("/rosbag2_player")]
    interfaces = [i for i in interfaces if i.name != "/clock"]
    interfaces = [i for i in interfaces if i.name != "/diagnostics"]
    interfaces = [i for i in interfaces if i.name != "/tf"]
    interfaces = [i for i in interfaces if i.name != "/tf_static"]
    for interface in interfaces:
        interface.classify()

    result = []
    for interface in interfaces:
        components = interface.components - {"rviz", "tier4_api"}
        if 2 <= len(components):
            result.append((interface.name, interface.types.pop()))
            # print(interface.name)
            # for node in interface.nodes:
            #     print(" -", node.type, node.component, node.name)

    for name in sorted(result, key=lambda t: t[0]):
        print("name: " + name[0])
        print("type: " + name[1])


if __name__ == "__main__":
    main()
