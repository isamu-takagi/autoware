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
    "system",
    "simulation",
    "adapi",
    "tier4_api",
    "rviz",
])

mapping = {
    "pose_initializer_node": "localization",
    "robot_state_publisher": "sensing",
    "occupancy_grid_map": "perception",
    "aggregator_node": "system",
    "static_map_to_odom_tf_publisher": "simulation",
    "default_ad_api": "adapi",
    "autoware_api": "tier4_api",
    "rviz2": "rviz",
}


def classify_component(name):
    component = name.split("/")[1]
    component = mapping.get(component, component)
    if component not in autoware_components:
        raise RuntimeError("unknown component: " + component)
    return component

def main():
    parser = ArgumentParser()
    parser.add_argument("interfaces")
    args = parser.parse_args()

    interfaces = yaml.safe_load(Path(args.interfaces).read_text())["links"]
    for interface in interfaces:
        nodes = []
        components = set()
        for node_type, node_name in interface["uses"]:
            if node_name.startswith("/transform_listener_impl_"):
                continue
            component = classify_component(node_name)
            components.add(component)
            nodes.append((node_type, component, node_name))
        if 2 <= len(components):
            print(interface["name"])
            for node in nodes:
                print(" -", *node)

if __name__ == "__main__":
    main()
