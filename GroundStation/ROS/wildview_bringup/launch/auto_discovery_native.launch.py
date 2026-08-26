import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node

# How often (seconds) to re-scan the network for newly joined drones after
# the initial launch, so a drone powered on later still gets picked up
# without restarting the whole ROS 2 stack.
DISCOVERY_PERIOD = float(os.environ.get("ROS_DISCOVERY_PERIOD", "15.0"))


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


def _scan_for_new_drones(discover_all_drones, known_namespaces, next_index, verbose=True):
    """Runs one discovery pass and returns Node actions for any drone not
    already launched. `known_namespaces`/`next_index` are mutated in place so
    repeated calls keep assigning stable, non-overlapping namespaces."""
    try:
        drones = discover_all_drones(timeout=5.0, verbose=verbose)
    except Exception as exc:
        print(f"Drone discovery failed: {exc}")
        return []

    actions = []
    for ip, name in drones:
        namespace = _namespace_for(name, next_index[0])
        if namespace in known_namespaces:
            continue
        print(f"Found drone: {name} at {ip} (namespace: {namespace})")
        actions.append(_create_dji_node(ip, name, next_index[0]))
        known_namespaces.add(namespace)
        next_index[0] += 1
    return actions


def _periodic_rescan(context, discover_all_drones, known_namespaces, next_index):
    actions = _scan_for_new_drones(discover_all_drones, known_namespaces, next_index, verbose=False)
    actions.append(
        TimerAction(
            period=DISCOVERY_PERIOD,
            actions=[
                OpaqueFunction(
                    function=lambda ctx: _periodic_rescan(
                        ctx, discover_all_drones, known_namespaces, next_index
                    )
                )
            ],
        )
    )
    return actions


def launch_setup(context, *args, **kwargs):
    try:
        discover_all_drones = _load_discovery_function()
    except ImportError:
        print("Could not import dji_controller.submodules.dji_interface.")
        return []

    print("Discovering drones...")
    known_namespaces = set()
    next_index = [0]
    actions = _scan_for_new_drones(discover_all_drones, known_namespaces, next_index)
    if not actions:
        print("No drones found via auto-discovery; will keep looking periodically.")

    # Keep discovering newly-joined drones after the initial launch instead
    # of only ever launching whatever this first scan happened to catch.
    actions.append(
        TimerAction(
            period=DISCOVERY_PERIOD,
            actions=[
                OpaqueFunction(
                    function=lambda ctx: _periodic_rescan(
                        ctx, discover_all_drones, known_namespaces, next_index
                    )
                )
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
