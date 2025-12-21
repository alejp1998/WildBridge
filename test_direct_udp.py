#!/usr/bin/env python3
"""
Test UDP communication directly to the phone's IP address.
This bypasses broadcast and tests if we can reach the phone at all.
"""
import socket
import sys

PHONE_IP = "192.168.0.16"
DISCOVERY_PORT = 30000
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"

def test_direct_udp():
    print(f"--- Direct UDP Test to {PHONE_IP}:{DISCOVERY_PORT} ---")
    
    try:
        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3.0)
        
        print(f"Sending discovery message directly to {PHONE_IP}...")
        sock.sendto(DISCOVERY_MSG, (PHONE_IP, DISCOVERY_PORT))
        
        # Wait for response
        print("Waiting for response...")
        try:
            data, addr = sock.recvfrom(1024)
            print(f"✅ SUCCESS! Response from {addr}:")
            print(f"   {data.decode('utf-8')}")
            return True
        except socket.timeout:
            print(f"❌ TIMEOUT - No response from phone.")
            print(f"\nPossible causes:")
            print(f"1. App not running or servers not started")
            print(f"2. Firewall blocking UDP on phone")
            print(f"3. App not listening on port {DISCOVERY_PORT}")
            print(f"4. Network isolation (AP isolation enabled)")
            return False
            
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False
    finally:
        sock.close()

def test_ping():
    """Test basic network connectivity"""
    print(f"\n--- Testing Network Connectivity ---")
    import subprocess
    
    print(f"Pinging {PHONE_IP}...")
    result = subprocess.run(['ping', '-c', '3', PHONE_IP], 
                          capture_output=True, 
                          text=True,
                          timeout=10)
    
    if result.returncode == 0:
        print("✅ Phone is reachable via ping")
        return True
    else:
        print("❌ Phone is NOT reachable via ping")
        print("This indicates a network connectivity issue.")
        return False

if __name__ == "__main__":
    # First test basic connectivity
    can_ping = test_ping()
    
    # Then test UDP
    print()
    success = test_direct_udp()
    
    if not can_ping and not success:
        print("\n⚠️  NETWORK ISSUE DETECTED")
        print("The phone is not reachable on the network.")
        print("Please verify:")
        print("- Phone and PC are on the same WiFi network")
        print("- Phone's IP is actually 192.168.0.16 (check phone WiFi settings)")
        print("- Router is not blocking communication between devices")
    elif can_ping and not success:
        print("\n⚠️  UDP PORT BLOCKED")
        print("The phone is reachable but not responding to UDP on port 30000.")
        print("Please verify:")
        print("- WildBridge app is running and shows 'Servers Started'")
        print("- App has network permissions")
        print("- Android firewall/security app is not blocking UDP")
    
    sys.exit(0 if success else 1)
