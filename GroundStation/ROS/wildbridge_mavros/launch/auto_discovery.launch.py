import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _load_discovery_function():
    try:
        from wildbridge_mavros.dji_interface import discover_all_drones

        return discover_all_drones
    except ImportError:
        current_dir = os.path.dirname(__file__)
        package_dir = os.path.abspath(os.path.join(current_dir, "../"))
        sys.path.append(package_dir) if package_dir not in sys.path else None
        from wildbridge_mavros.dji_interface import discover_all_drones

        return discover_all_drones


def _namespace_for(name, index):
    clean_name = "".join(c if c.isalnum() or c == "_" else "_" for c in name)
    return f"drone_{index + 1}" if not clean_name or clean_name == "UNKNOWN" else clean_name


def _create_mavros_node(ip, name, index):
    namespace = _namespace_for(name, index)
    return Node(
        package="wildbridge_mavros",
        executable="mavros_bridge",
        name=f"wildbridge_mavros_{namespace}",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "drone_ip": ip,
                "namespace_mode": "manual",
                "system_id": index + 1,
                "component_id": 1,
            }
        ],
    )


def launch_setup(context, *args, **kwargs):
    try:
        discover_all_drones = _load_discovery_function()
    except ImportError:
        print("Could not import wildbridge_mavros.dji_interface.")
        return []

    print("Discovering drones...")
    drones = discover_all_drones(timeout=5.0)

    if not drones:
        print("No drones found via auto-discovery.")
        return []

    actions = []
    for index, (ip, name) in enumerate(drones):
        print(f"Found drone: {name} at {ip}")
        actions.append(_create_mavros_node(ip, name, index))

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
