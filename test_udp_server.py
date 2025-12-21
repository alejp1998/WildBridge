#!/usr/bin/env python3
"""
Simple UDP echo server to test if we can receive packets on port 30000
This simulates what the Android app should be doing.
"""
import socket

DISCOVERY_PORT = 30000

def run_udp_server():
    print(f"Starting UDP test server on port {DISCOVERY_PORT}...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', DISCOVERY_PORT))
        
        print(f"✅ Server listening on 0.0.0.0:{DISCOVERY_PORT}")
        print("Waiting for packets... (Ctrl+C to stop)")
        
        while True:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8', errors='ignore')
            print(f"\n📦 Received from {addr}:")
            print(f"   {message}")
            
            # Echo back
            response = f"ECHO: {message}"
            sock.sendto(response.encode(), addr)
            print(f"   Sent response: {response}")
            
    except KeyboardInterrupt:
        print("\n\nServer stopped.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    run_udp_server()
