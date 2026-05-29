"""
WildBridge - DJI Interface Module

A Python interface for controlling DJI drones through HTTP requests and TCP sockets,
providing seamless integration for drone operations, telemetry retrieval, and video streaming.

Authors: Edouard G.A. Rolland, Kilian Meier, Alejandro Jarabo-Peñas
Project: WildDrone
Institution: University of Bristol, University of Southern Denmark (SDU)
License: MIT

For more information, visit: https://github.com/WildDrone/WildBridge
"""

import json
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import suppress
from enum import Enum
from pathlib import Path

import requests
from wildbridge_groundstation.dji_client import DJIInterface as _SharedDJIInterface
from wildbridge_groundstation.dji_client import get_config as _shared_get_config
from wildbridge_groundstation.dji_helpers import (
    candidate_subnet_ips,
    parse_discovery_response_tuple,
)

# Discovery Configuration
DISCOVERY_PORT = 30000
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"
MULTICAST_GROUP = "239.255.42.99"  # Multicast address for drone discovery
MULTICAST_PORT = 30001

# mDNS/Zeroconf Configuration
MDNS_SERVICE_TYPE = "_wildbridge._tcp.local."
MDNS_SERVICE_NAME = "WildBridge Drone"

# Cache configuration
CACHE_DIR = Path.home() / ".wildbridge"
CACHE_FILE = CACHE_DIR / "drones_cache.json"
CACHE_MAX_AGE = 3600  # 1 hour
# Optional: Try to import zeroconf for mDNS support
try:
    from zeroconf import ServiceBrowser, Zeroconf

    ZEROCONF_AVAILABLE = True
except ImportError:
    ZEROCONF_AVAILABLE = False
    print("Note: zeroconf library not installed. Install with: pip install zeroconf")


class DroneState(Enum):
    """Drone connection states for health monitoring."""

    DISCOVERING = 1
    CONNECTED = 2
    DISCONNECTED = 3
    RECONNECTING = 4
    FAILED = 5


class DiscoveryCache:
    """Persistent cache for discovered drones."""

    def __init__(self, cache_file=CACHE_FILE):
        self.cache_file = Path(cache_file)
        self.cache_file.parent.mkdir(parents=True, exist_ok=True)
        self.drones = self.load()

    def load(self):
        """Load cache from disk."""
        if not self.cache_file.exists():
            return {}
        try:
            with open(self.cache_file) as f:
                return json.load(f)
        except Exception as e:
            print(f"Failed to load drone cache: {e}")
            return {}

    def save(self):
        """Save cache to disk."""
        try:
            with open(self.cache_file, "w") as f:
                json.dump(self.drones, f, indent=2)
        except Exception as e:
            print(f"Failed to save drone cache: {e}")

    def update(self, ip, name, metadata=None):
        """Update cache with new or existing drone."""
        self.drones[ip] = {"name": name, "last_seen": time.time(), "metadata": metadata or {}}
        self.save()

    def get_recent(self, max_age=CACHE_MAX_AGE):
        """Get recently seen drones (within max_age seconds)."""
        cutoff = time.time() - max_age
        recent = {}
        for ip, data in self.drones.items():
            if data.get("last_seen", 0) > cutoff:
                recent[ip] = data
        return recent

    def get_all(self):
        """Get all cached drones."""
        return self.drones.copy()

    def remove(self, ip):
        """Remove a drone from cache."""
        if ip in self.drones:
            del self.drones[ip]
            self.save()
            return True
        return False

    def clear(self):
        """Clear all cache."""
        self.drones = {}
        self.save()


class DroneHealthMonitor:
    """Monitor drone health with automatic reconnection."""

    def __init__(self, check_interval=5.0):
        self.check_interval = check_interval
        self.monitors = {}
        self._running = False

    def start_monitoring(self, ip, name, on_disconnect=None, on_reconnect=None):
        """Start monitoring a drone."""
        if ip in self.monitors:
            return

        monitor_info = {
            "ip": ip,
            "name": name,
            "state": DroneState.CONNECTED,
            "consecutive_failures": 0,
            "on_disconnect": on_disconnect,
            "on_reconnect": on_reconnect,
            "thread": None,
        }

        self.monitors[ip] = monitor_info

        # Start monitoring thread
        thread = threading.Thread(target=self._monitor_drone, args=(ip,), daemon=True)
        monitor_info["thread"] = thread
        thread.start()

    def stop_monitoring(self, ip):
        """Stop monitoring a drone."""
        if ip in self.monitors:
            del self.monitors[ip]

    def stop_all(self):
        """Stop all monitoring."""
        self.monitors.clear()

    def _monitor_drone(self, ip):
        """Background monitoring thread."""
        while ip in self.monitors:
            monitor = self.monitors[ip]

            try:
                # Try to ping the config endpoint
                response = requests.get(f"http://{ip}:8080/config", timeout=2.0)

                if response.status_code == 200:
                    # Drone is healthy
                    if monitor["state"] != DroneState.CONNECTED:
                        # Reconnected!
                        monitor["state"] = DroneState.CONNECTED
                        monitor["consecutive_failures"] = 0
                        if monitor["on_reconnect"]:
                            monitor["on_reconnect"](ip, monitor["name"])
                else:
                    self._handle_failure(ip, monitor)

            except Exception:
                self._handle_failure(ip, monitor)

            time.sleep(self.check_interval)

    def _handle_failure(self, ip, monitor):
        """Handle drone connection failure."""
        monitor["consecutive_failures"] += 1

        if monitor["consecutive_failures"] == 1:
            # First failure, mark as disconnected
            monitor["state"] = DroneState.DISCONNECTED
            if monitor["on_disconnect"]:
                monitor["on_disconnect"](ip, monitor["name"])

        if monitor["consecutive_failures"] > 5:
            monitor["state"] = DroneState.FAILED

    def get_state(self, ip):
        """Get current state of a drone."""
        if ip in self.monitors:
            return self.monitors[ip]["state"]
        return None


def get_local_ips():
    """Get all local IP addresses and network information."""
    ip_list = []
    try:
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if not ip.startswith("127."):
                ip_list.append(ip)
    except OSError:
        pass

    # Fallback method
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip not in ip_list and not ip.startswith("127."):
            ip_list.append(ip)
    except OSError:
        pass

    return ip_list


def get_broadcast_addresses():
    """Get all possible broadcast addresses for local networks."""
    broadcast_addrs = ["255.255.255.255"]  # Global broadcast

    local_ips = get_local_ips()
    for ip in local_ips:
        parts = ip.split(".")
        # Assume /24 subnet for simplicity (most common)
        subnet_broadcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
        if subnet_broadcast not in broadcast_addrs:
            broadcast_addrs.append(subnet_broadcast)

        # Also try /16 subnet broadcast
        class_b_broadcast = f"{parts[0]}.{parts[1]}.255.255"
        if class_b_broadcast not in broadcast_addrs:
            broadcast_addrs.append(class_b_broadcast)

    return broadcast_addrs


def _parse_discovery_response(data, addr):
    return parse_discovery_response_tuple(data, fallback_ip=addr[0])


def _remember_discovered_drone(found_drones, discovery, verbose, message_template):
    if not discovery:
        return
    drone_ip, drone_name = discovery
    if drone_ip in found_drones:
        return
    if verbose:
        print(message_template.format(ip=drone_ip, name=drone_name))
    found_drones[drone_ip] = drone_name


def _collect_discovery_responses(sock, timeout, found_drones, verbose, message_template):
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            data, addr = sock.recvfrom(1024)
        except TimeoutError:
            break
        discovery = _parse_discovery_response(data, addr)
        _remember_discovered_drone(found_drones, discovery, verbose, message_template)


def _close_socket_quietly(sock):
    with suppress(OSError):
        sock.close()


def _candidate_subnet_ips(local_ips):
    return candidate_subnet_ips(local_ips)


def _create_multicast_socket(timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    sock.bind(("", MULTICAST_PORT))
    mreq = socket.inet_aton(MULTICAST_GROUP) + socket.inet_aton("0.0.0.0")  # nosec B104
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(timeout)
    return sock


def _create_broadcast_socket(timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", DISCOVERY_PORT + 1))
    except OSError:
        sock.bind(("", 0))
    sock.settimeout(timeout)
    return sock


def discover_via_multicast(timeout=3.0, verbose=True):
    """
    Discover drones using IP multicast - works better across VLANs and larger networks.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    sock = None

    try:
        sock = _create_multicast_socket(timeout)
        sock.sendto(DISCOVERY_MSG, (MULTICAST_GROUP, MULTICAST_PORT))
        if verbose:
            print(f"Sent multicast discovery to {MULTICAST_GROUP}:{MULTICAST_PORT}")
        _collect_discovery_responses(
            sock,
            timeout,
            found_drones,
            verbose,
            "✓ Found drone via multicast: {ip} (Name: {name})",
        )
    except Exception as e:
        if verbose:
            print(f"Multicast discovery failed: {e}")
    finally:
        if sock:
            _close_socket_quietly(sock)

    return list(found_drones.items())


class WildBridgeServiceListener:
    """Listener for mDNS/Zeroconf service discovery."""

    def __init__(self, verbose=True):
        self.found_drones = {}
        self.verbose = verbose

    def add_service(self, zc, type_, name):
        """Called when a service is discovered."""
        info = zc.get_service_info(type_, name)
        if info and info.addresses:
            ip = socket.inet_ntoa(info.addresses[0])
            drone_name = info.properties.get(b"name", b"").decode("utf-8")
            if not drone_name:
                drone_name = name.split(".")[0]

            if ip not in self.found_drones:
                self.found_drones[ip] = drone_name
                if self.verbose:
                    print(f"  ✓ mDNS: Found {drone_name} at {ip}")

    def remove_service(self, zc, type_, name):
        """Called when a service is removed."""
        pass

    def update_service(self, zc, type_, name):
        """Called when a service is updated."""
        self.add_service(zc, type_, name)


def discover_via_mdns(timeout=5.0, verbose=True):
    """
    Discover drones using mDNS/Zeroconf - the industry standard for local service discovery.
    Works across subnets (with mDNS repeaters), more reliable than UDP broadcast.

    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    if not ZEROCONF_AVAILABLE:
        if verbose:
            print("  ⚠ mDNS not available (install: pip install zeroconf)")
        return []

    found_drones = {}

    try:
        zeroconf = Zeroconf()
        listener = WildBridgeServiceListener(verbose=verbose)

        # Browse for WildBridge services
        browser = ServiceBrowser(zeroconf, MDNS_SERVICE_TYPE, listener)

        if verbose:
            print(f"  Browsing for {MDNS_SERVICE_TYPE}...")

        # Wait for discovery
        time.sleep(timeout)

        # Get results
        found_drones = listener.found_drones.copy()

        # Cleanup
        browser.cancel()
        zeroconf.close()

    except Exception as e:
        if verbose:
            print(f"  ✗ mDNS discovery failed: {e}")

    return list(found_drones.items())


def discover_via_broadcast_enhanced(timeout=3.0, verbose=True):
    """
    Enhanced broadcast discovery trying multiple broadcast addresses and socket configurations.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    broadcast_addrs = get_broadcast_addresses()

    if verbose:
        print(f"Trying broadcast discovery on {len(broadcast_addrs)} address(es)...")

    for broadcast_addr in broadcast_addrs:
        sock = None
        try:
            sock = _create_broadcast_socket(timeout)
            sock.sendto(DISCOVERY_MSG, (broadcast_addr, DISCOVERY_PORT))
            if verbose:
                print(f"  → Broadcast to {broadcast_addr}:{DISCOVERY_PORT}")
            _collect_discovery_responses(
                sock,
                timeout,
                found_drones,
                verbose,
                "  ✓ Response from {ip} (Name: {name})",
            )
        except Exception as e:
            if verbose:
                print(f"  ✗ Broadcast to {broadcast_addr} failed: {e}")
        finally:
            if sock:
                _close_socket_quietly(sock)

    return list(found_drones.items())


def probe_single_ip(ip, timeout=1.5):
    """
    Probe a single IP for WildBridge drone.
    Returns (ip, name) tuple or None.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        sock.sendto(DISCOVERY_MSG, (ip, DISCOVERY_PORT))
        try:
            data, addr = sock.recvfrom(1024)
            return _parse_discovery_response(data, addr)
        except TimeoutError:
            pass
    except OSError:
        pass
    finally:
        if "sock" in locals():
            _close_socket_quietly(sock)

    return None


def scan_subnet_for_drones_parallel(local_ips, timeout=1.5, verbose=True, max_workers=50):
    """
    Scan subnet for WildBridge drones using parallel UDP probing.
    Much faster than sequential scanning.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}

    if verbose:
        print("Scanning subnet for WildBridge drones (parallel mode)...")

    ips_to_scan = _candidate_subnet_ips(local_ips)

    # Probe all IPs in parallel
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_ip = {executor.submit(probe_single_ip, ip, timeout): ip for ip in ips_to_scan}

        for future in as_completed(future_to_ip):
            result = future.result()
            if result:
                drone_ip, drone_name = result
                if drone_ip not in found_drones:
                    found_drones[drone_ip] = drone_name
                    if verbose:
                        print(f"✓ Found WildBridge drone at {drone_ip} (Name: {drone_name})")

    return list(found_drones.items())


def scan_subnet_for_drones(local_ips, timeout=0.1, verbose=True):
    """
    Scan subnet for WildBridge drones using direct UDP probing.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    if verbose:
        print("Scanning subnet for WildBridge drones...")

    for ip in _candidate_subnet_ips(local_ips):
        discovery = probe_single_ip(ip, timeout)
        _remember_discovered_drone(
            found_drones,
            discovery,
            verbose,
            "Found WildBridge drone at {ip} (Name: {name})",
        )

    return list(found_drones.items())


def verify_drone_http(ip, timeout=1.0):
    """Verify drone is running by checking the HTTP config endpoint."""
    try:
        response = requests.get(f"http://{ip}:8080/config", timeout=timeout)
    except requests.exceptions.RequestException:
        return False
    return response.status_code == 200


def _verify_cached_drones(cache, found_drones, failed_cache_entries, verbose):
    recent = cache.get_recent()
    if not recent:
        return

    if verbose:
        print(f"[Cache] Found {len(recent)} drone(s) in cache, verifying...")

    for ip, data in recent.items():
        name = data.get("name", "UNKNOWN")
        if verify_drone_http(ip):
            found_drones[ip] = name
            if verbose:
                print(f"  ✓ Verified {ip} (Name: {name})")
            continue

        failed_cache_entries[ip] = name
        if verbose:
            print(f"  ✗ {ip} ({name}) not responding - will search for new IP")


def _remove_rehomed_cache_entry(cache, failed_cache_entries, name, new_ip, verbose):
    if not cache:
        return

    for old_ip, old_name in list(failed_cache_entries.items()):
        if old_name != name:
            continue
        if verbose:
            print(f"  ↳ {name} IP changed: {old_ip} → {new_ip}")
        cache.remove(old_ip)
        del failed_cache_entries[old_ip]
        return


def _add_verified_drones(
    candidates, found_drones, failed_cache_entries, cache, verbose, inactive_message
):
    new_count = 0
    for ip, name in candidates:
        if ip in found_drones:
            continue
        if not verify_drone_http(ip):
            if verbose:
                print(inactive_message.format(ip=ip))
            continue

        found_drones[ip] = name
        new_count += 1
        if cache:
            cache.update(ip, name)
        _remove_rehomed_cache_entry(cache, failed_cache_entries, name, ip, verbose)
        if verbose:
            print(f"  ✓ Verified {ip} (Name: {name})")
    return new_count


def _print_discovery_count(label, new_count, found_drones, verbose):
    if not verbose:
        return
    if new_count > 0:
        print(f"[{label}] ✓ Found {new_count} verified drone(s) (total: {len(found_drones)})")
    else:
        print(f"[{label}] No new active drones found")


def _discover_via_mdns(found_drones, failed_cache_entries, cache, timeout, verbose):
    if not ZEROCONF_AVAILABLE:
        return
    if verbose:
        print("[mDNS] Trying mDNS/Zeroconf discovery...")

    mdns_drones = discover_via_mdns(timeout=min(timeout, 5.0), verbose=verbose)
    new_count = _add_verified_drones(
        mdns_drones,
        found_drones,
        failed_cache_entries,
        cache,
        verbose,
        "  ✗ {ip} mDNS registered but app not running",
    )
    _print_discovery_count("mDNS", new_count, found_drones, verbose)


def _discover_via_subnet_scan(found_drones, failed_cache_entries, cache, verbose):
    if verbose:
        print("[Subnet Scan] Scanning local subnets for drones...")

    local_ips = get_local_ips()
    if not local_ips:
        return
    if verbose:
        print(f"  Local IPs detected: {', '.join(local_ips)}")

    subnet_drones = scan_subnet_for_drones_parallel(
        local_ips,
        timeout=1.5,
        verbose=verbose,
        max_workers=50,
    )
    new_count = _add_verified_drones(
        subnet_drones,
        found_drones,
        failed_cache_entries,
        cache,
        verbose,
        "  ✗ {ip} responded to UDP but app not running",
    )
    _print_discovery_count("Subnet Scan", new_count, found_drones, verbose)


def _remove_stale_cache_entries(cache, failed_cache_entries, verbose):
    if not cache:
        return
    for ip, name in failed_cache_entries.items():
        cache.remove(ip)
        if verbose:
            print(f"[Cache] Removed stale entry for {name} at {ip} (not found on network)")


def _print_discovery_summary(found_drones, verbose):
    if not verbose:
        return
    if found_drones:
        print(f"✓ Discovery complete: Found {len(found_drones)} active drone(s)")
        return

    print("✗ No active drones found with any method")
    print("\nTroubleshooting tips:")
    print("  1. Ensure drone WildBridge app is running")
    print("  2. Check firewall allows UDP port 30000-30001 and TCP port 8080")
    print("  3. Verify you're on the same network as the drone")
    print("  4. Try: sudo iptables -L to check for blocking rules")


def discover_all_drones(timeout=5.0, verbose=True, use_cache=True, max_retries=2):
    """
    Discover all WildBridge drones on the network with enhanced robustness.

    Discovery methods (in order of preference):
    1. Cache verification (fastest - instant if drone was seen recently)
    2. mDNS/Zeroconf (most reliable, industry standard)
    3. Targeted subnet scan (always runs to find drones not in cache/mDNS)

    Args:
        timeout: Timeout for each discovery method
        verbose: Print discovery progress
        use_cache: Try cached drones first
        max_retries: Number of retry attempts (unused, kept for API compatibility)

    Returns:
        List of tuples [(drone_ip, drone_name), ...]
    """
    del max_retries

    found_drones = {}
    failed_cache_entries = {}
    cache = DiscoveryCache() if use_cache else None

    if cache:
        _verify_cached_drones(cache, found_drones, failed_cache_entries, verbose)
    if found_drones and verbose:
        print(
            f"[Cache] Verified {len(found_drones)} cached drone(s), continuing to search for more..."
        )

    _discover_via_mdns(found_drones, failed_cache_entries, cache, timeout, verbose)
    _discover_via_subnet_scan(found_drones, failed_cache_entries, cache, verbose)
    _remove_stale_cache_entries(cache, failed_cache_entries, verbose)
    _print_discovery_summary(found_drones, verbose)

    return list(found_drones.items())


def discover_drone(timeout=5.0, verbose=True):
    """
    Discover a single WildBridge drone.
    Returns tuple (drone_ip, drone_name) or (None, None).
    """
    drones = discover_all_drones(timeout, verbose)
    if drones:
        return drones[0]
    return None, None


# HTTP POST Command Endpoints (port 8080)
EP_STICK = "/send/stick"  # expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_GIMBAL_SET_YAW = "/send/gimbal/yaw"  # !!! This is the yaw joint angle !!!
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_ABORT_ALL = "/send/abortAll"
EP_GOTO_WP = "/send/gotoWP"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_PID = "/send/gotoWPwithPID"
EP_GOTO_TRAJECTORY = "/send/navigateTrajectory"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"
EP_GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative"
EP_ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission"
EP_SET_RTH_ALTITUDE = "/send/setRTHAltitude"
EP_DEACTIVATE_MANUAL_OVERRIDE = "/send/deactivateManualOverride"

# PID Tuning
EP_TUNING = "/send/gotoWPwithPIDtuning"


get_config = _shared_get_config


class DJIInterface(_SharedDJIInterface):
    """Backward-compatible MAVROS DJI client using the shared GroundStation implementation."""

    def __init__(self, IP_RC=""):
        super().__init__(
            IP_RC,
            discover_callback=discover_drone,
            config_loader=get_config,
            query_config_name=True,
        )


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
