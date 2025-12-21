import socket
import time
import sys

DISCOVERY_PORT = 30000
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"
DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"

def get_ip_addresses():
    ip_list = []
    try:
        # Get all hostnames and IPs
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if not ip.startswith("127."):
                ip_list.append(ip)
    except:
        pass
    
    # Fallback/Additional method using socket connection trick
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip not in ip_list and not ip.startswith("127."):
            ip_list.append(ip)
    except:
        pass
        
    return ip_list

def debug_discovery():
    print(f"--- WildBridge Discovery Debugger ---")
    print(f"Listening for responses on port {DISCOVERY_PORT}")
    
    interfaces = get_ip_addresses()
    print(f"Detected Local IPs: {interfaces}")
    
    found = False
    
    # Try global broadcast first
    try:
        print("\n[Attempt 1] Sending global broadcast to 255.255.255.255...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(2.0)
        sock.sendto(DISCOVERY_MSG, ('255.255.255.255', DISCOVERY_PORT))
        
        try:
            data, addr = sock.recvfrom(1024)
            print(f"✅ RESPONSE RECEIVED from {addr}: {data.decode('utf-8')}")
            found = True
        except socket.timeout:
            print("❌ No response.")
        finally:
            sock.close()
    except Exception as e:
        print(f"Error: {e}")

    # Try subnet broadcasts for each interface
    for ip in interfaces:
        if found: break
        
        # Estimate broadcast address (assuming /24 subnet for simplicity)
        parts = ip.split('.')
        broadcast_ip = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
        
        print(f"\n[Attempt 2] Sending broadcast to {broadcast_ip} (via {ip})...")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            # Bind to the specific interface IP to ensure it goes out that way
            sock.bind((ip, 0)) 
            sock.settimeout(2.0)
            sock.sendto(DISCOVERY_MSG, (broadcast_ip, DISCOVERY_PORT))
            
            try:
                data, addr = sock.recvfrom(1024)
                print(f"✅ RESPONSE RECEIVED from {addr}: {data.decode('utf-8')}")
                found = True
            except socket.timeout:
                print("❌ No response.")
        except Exception as e:
            print(f"Error on interface {ip}: {e}")
        finally:
            sock.close()

    if not found:
        print("\n⚠️  Discovery failed on all attempts.")
        print("Troubleshooting:")
        print("1. Ensure phone and computer are on the SAME WiFi network.")
        print("2. Check if 'AP Isolation' is enabled on your router.")
        print("3. Disable computer firewall temporarily.")
        print("4. Verify the app shows 'WildBridge Servers Started'.")
        
        # Try scanning common IPs in the subnet
        print("\n[Fallback] Attempting to scan subnet for WildBridge...")
        found_device = scan_subnet(interfaces)
        if found_device:
            print(f"\n✅ Found WildBridge at {found_device}")
        else:
            print("\n❌ No WildBridge devices found on network.")

def scan_subnet(interfaces):
    """Scan common IP addresses in the subnet looking for WildBridge"""
    for local_ip in interfaces:
        parts = local_ip.split('.')
        subnet = f"{parts[0]}.{parts[1]}.{parts[2]}"
        
        print(f"Scanning {subnet}.0/24...")
        
        # Try common IP addresses (1-20, 50-70, 100-120, 150-170, 200-220)
        ranges = list(range(1, 21)) + list(range(50, 71)) + list(range(100, 121)) + list(range(150, 171)) + list(range(200, 221))
        
        for i in ranges:
            ip = f"{subnet}.{i}"
            if ip == local_ip:  # Skip our own IP
                continue
                
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.1)  # Fast timeout for scanning
                sock.sendto(DISCOVERY_MSG, (ip, DISCOVERY_PORT))
                
                try:
                    data, addr = sock.recvfrom(1024)
                    if data.decode('utf-8').startswith(DISCOVERY_RESPONSE_PREFIX):
                        sock.close()
                        return addr[0]
                except socket.timeout:
                    pass
                
                sock.close()
            except:
                pass
    
    return None

if __name__ == "__main__":
    debug_discovery()
