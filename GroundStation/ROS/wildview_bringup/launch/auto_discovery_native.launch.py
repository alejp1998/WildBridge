import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _load_discovery_function():
    try:
        from dji_controller.submodules.dji_interface import discover_all_drones

        return discover_all_drones
    except ImportError:
        current_dir = os.path.dirname(__file__)
        ros_dir = os.path.abspath(os.path.join(current_dir, "../../"))
        dji_controller_path = os.path.join(ros_dir, "dji_controller")
        sys.path.append(dji_controller_path) if dji_controller_path not in sys.path else None
        from dji_controller.submodules.dji_interface import discover_all_drones

        return discover_all_drones


def _namespace_for(name, index):
    clean_name = "".join(c if c.isalnum() or c == "_" else "_" for c in name)
    return f"drone_{index + 1}" if not clean_name or clean_name == "UNKNOWN" else clean_name


def _create_dji_node(ip, name, index):
    namespace = _namespace_for(name, index)
    return Node(
        package="dji_controller",
        executable="dji_node",
        name=f"dji_node_{namespace}",
        namespace=namespace,
        output="screen",
        parameters=[{"ip_rc": ip}],
    )


def launch_setup(context, *args, **kwargs):
    try:
        discover_all_drones = _load_discovery_function()
    except ImportError:
        print("Could not import dji_controller.submodules.dji_interface.")
        return []

    print("Discovering drones...")
    drones = discover_all_drones(timeout=5.0)

    if not drones:
        print("No drones found via auto-discovery.")
        return []

    actions = []
    for index, (ip, name) in enumerate(drones):
        print(f"Found drone: {name} at {ip}")
        actions.append(_create_dji_node(ip, name, index))

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
