#!/usr/bin/env python3
import sys
import os
import time
import subprocess

# Add Python directory to path to import djiInterface
sys.path.append(os.path.join(os.path.dirname(__file__), 'Python'))

try:
    from djiInterface import discover_drone
except ImportError:
    print("Error: Could not import djiInterface. Make sure you are running this from the GroundStation directory.")
    sys.exit(1)

def main():
    print("Starting WildBridge Ground Station Integration Test...")
    print("1. Attempting to discover drone via UDP broadcast...")
    
    # Try to discover drone
    drone_ip = discover_drone(timeout=10.0)
    
    if not drone_ip:
        print("❌ Failed to discover drone. Make sure:")
        print("   - The Android app is running")
        print("   - The phone and computer are on the same network")
        print("   - UDP broadcast (port 30000) is allowed")
        sys.exit(1)
        
    print(f"✅ Drone discovered at IP: {drone_ip}")
    
    # Construct WebRTC URL
    webrtc_url = f"ws://{drone_ip}:8082"
    print(f"2. Launching WebRTC Viewer connecting to {webrtc_url}...")
    
    # Path to viewer script
    viewer_script = os.path.join(os.path.dirname(__file__), 'webrtc_client', 'webrtc_drone_viewer.py')
    
    # Command to run viewer
    cmd = [sys.executable, viewer_script, '--server', webrtc_url]
    
    print(f"Executing: {' '.join(cmd)}")
    
    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    except subprocess.CalledProcessError as e:
        print(f"❌ Viewer exited with error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
