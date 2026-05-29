"""
WildBridge - DJI Interface Module

ROS dji_controller compatibility wrapper around the shared GroundStation DJI client.

Authors: Edouard G.A. Rolland, Kilian Meier, Alejandro Jarabo-Peñas
Project: WildDrone
Institution: University of Bristol, University of Southern Denmark (SDU)
License: MIT

For more information, visit: https://github.com/WildDrone/WildBridge
"""

import socket
import time
from contextlib import suppress

from wildbridge_groundstation.dji_client import *  # noqa: F403
from wildbridge_groundstation.dji_client import DISCOVERY_MSG, DISCOVERY_PORT
from wildbridge_groundstation.dji_client import DJIInterface as _SharedDJIInterface
from wildbridge_groundstation.dji_helpers import (
    candidate_subnet_ips,
    parse_discovery_response_tuple,
)


def _close_socket_quietly(sock):
    with suppress(OSError):
        sock.close()


def _parse_discovery_response(data, addr):
    return parse_discovery_response_tuple(data, fallback_ip=addr[0])


def _remember_discovered_drone(found_drones, discovery, verbose):
    if not discovery:
        return
    drone_ip, drone_name = discovery
    if drone_ip in found_drones:
        return
    if verbose:
        print(f"Found WildBridge drone at {drone_ip} (Name: {drone_name})")
    found_drones[drone_ip] = drone_name


def _candidate_subnet_ips(local_ips):
    return candidate_subnet_ips(local_ips)


def _probe_single_ip(ip, timeout):
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        sock.sendto(DISCOVERY_MSG, (ip, DISCOVERY_PORT))
        data, addr = sock.recvfrom(1024)
        return _parse_discovery_response(data, addr)
    except (TimeoutError, OSError):
        return None
    finally:
        if sock:
            _close_socket_quietly(sock)


def _collect_broadcast_responses(sock, timeout, found_drones, verbose):
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            data, addr = sock.recvfrom(1024)
        except TimeoutError:
            continue
        _remember_discovered_drone(
            found_drones,
            _parse_discovery_response(data, addr),
            verbose,
        )


def get_local_ips():
    """Get all local IP addresses for subnet detection."""
    ip_list = []
    try:
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if not ip.startswith("127."):
                ip_list.append(ip)
    except OSError:
        pass

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        if ip not in ip_list and not ip.startswith("127."):
            ip_list.append(ip)
    except OSError:
        pass

    return ip_list


def scan_subnet_for_drones(local_ips, timeout=0.1, verbose=True):
    """Scan subnet for WildBridge drones using direct UDP probing."""
    found_drones = {}
    if verbose:
        print("Scanning subnet for WildBridge drones...")

    for ip in _candidate_subnet_ips(local_ips):
        _remember_discovered_drone(found_drones, _probe_single_ip(ip, timeout), verbose)

    return list(found_drones.items())


def discover_all_drones(timeout=5.0, verbose=True):
    """Discover all WildBridge drones on the network."""
    found_drones = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(timeout)

    try:
        sock.sendto(DISCOVERY_MSG, ("<broadcast>", DISCOVERY_PORT))
        if verbose:
            print(f"Broadcasting discovery message on port {DISCOVERY_PORT}...")
        _collect_broadcast_responses(sock, timeout, found_drones, verbose)
    except Exception as exc:
        if verbose:
            print(f"Broadcast discovery failed: {exc}")
    finally:
        _close_socket_quietly(sock)

    if not found_drones:
        if verbose:
            print("Broadcast found no drones, scanning subnet...")
        local_ips = get_local_ips()
        if local_ips:
            for ip, name in scan_subnet_for_drones(local_ips, timeout=0.1, verbose=verbose):
                found_drones[ip] = name

    return list(found_drones.items())


def discover_drone(timeout=5.0, verbose=True):
    """Discover a single WildBridge drone."""
    drones = discover_all_drones(timeout, verbose)
    if drones:
        return drones[0]
    return None, None


class DJIInterface(_SharedDJIInterface):
    """Backward-compatible ROS dji_controller DJI client."""

    def __init__(self, IP_RC=""):
        super().__init__(IP_RC, discover_callback=discover_drone)


if __name__ == "__main__":
    import sys

    IP_RC = "10.102.252.30"
    if len(sys.argv) > 1:
        IP_RC = sys.argv[1]

    print(f"Connecting to {IP_RC}...")
    dji = DJIInterface(IP_RC)
    print("Starting telemetry stream...")
    dji.startTelemetryStream()
    try:
        while True:
            telemetry = dji.getTelemetry()
            print(telemetry or "Waiting for telemetry data...")
            time.sleep(0.1)
    except KeyboardInterrupt:
        dji.stopTelemetryStream()
