#!/usr/bin/env python3
"""
WebRTC Drone Video Viewer

This script connects to the DJI drone's WebRTC server and displays the video feed.
It can also save frames or the video stream to disk.

Requirements:
    pip install aiortc aiohttp opencv-python numpy websockets

Usage:
    python webrtc_drone_viewer.py --server ws://192.168.1.100:8081
    python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --save-video output.mp4
    python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --headless --save-frames ./frames/
"""

import argparse
import asyncio
import json
import logging
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRecorder
from av import VideoFrame

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class DroneVideoReceiver:
    """Handles WebRTC connection and video frame processing."""
    
    def __init__(self, server_url: str, headless: bool = False, 
                 save_video: str = None, save_frames: str = None):
        self.server_url = server_url
        self.headless = headless
        self.save_video = save_video
        self.save_frames = save_frames
        
        self.pc = None
        self.websocket = None
        self.client_id = None
        self.connected = False
        self.frame_count = 0
        self.start_time = None
        self.latest_frame = None
        self.recorder = None
        
        # Telemetry data from data channel
        self.latest_metadata = None
        self.metadata_lock = asyncio.Lock()
        
        # Create frames directory if saving frames
        if self.save_frames and not os.path.exists(self.save_frames):
            os.makedirs(self.save_frames)
    
    async def connect(self):
        """Connect to the signaling server and establish WebRTC connection."""
        logger.info(f"Connecting to signaling server: {self.server_url}")
        
        try:
            self.websocket = await websockets.connect(self.server_url)
            logger.info("WebSocket connected")
            
            # Wait for welcome message
            welcome = await self.websocket.recv()
            welcome_data = json.loads(welcome)
            if welcome_data.get('type') == 'welcome':
                self.client_id = welcome_data.get('clientId')
                logger.info(f"Assigned client ID: {self.client_id}")
            
            # Register as viewer
            await self.websocket.send(json.dumps({
                'type': 'register',
                'role': 'viewer'
            }))
            
            # Create peer connection
            self.pc = RTCPeerConnection()
            
            # Create data channel for metadata with same settings as Android side
            try:
                # Use negotiated mode with specific ID to match Android side
                self.metadata_channel = self.pc.createDataChannel(
                    "telemetry", 
                    negotiated=True,
                    id=0,
                    ordered=True
                )
                logger.info("Created metadata data channel (negotiated mode)")
                
                @self.metadata_channel.on("message")
                def on_metadata_message(message):
                    try:
                        self.latest_metadata = json.loads(message)
                        logger.debug(f"Received metadata: frame {self.latest_metadata.get('frameNumber', 'N/A')}")
                    except json.JSONDecodeError as e:
                        logger.error(f"Failed to parse metadata: {e}")
            except Exception as e:
                logger.error(f"Error creating metadata channel: {e}")
            
            # Handle incoming data channel for metadata (if Android creates it differently)
            @self.pc.on("datachannel")
            def on_datachannel(channel):
                logger.info(f"Received data channel: {channel.label}")
                if channel.label == "telemetry":
                    logger.info("Metadata data channel received from peer")
                    @channel.on("message")
                    def on_message(message):
                        try:
                            self.latest_metadata = json.loads(message)
                            logger.debug(f"Received metadata from peer: frame {self.latest_metadata.get('frameNumber', 'N/A')}")
                        except json.JSONDecodeError as e:
                            logger.error(f"Failed to parse metadata: {e}")
            
            # Handle incoming tracks
            @self.pc.on("track")
            def on_track(track):
                logger.info(f"Received track: {track.kind}")
                if track.kind == "video":
                    asyncio.ensure_future(self._handle_video_track(track))
            
            # Handle connection state changes
            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                logger.info(f"Connection state: {self.pc.connectionState}")
                if self.pc.connectionState == "connected":
                    self.connected = True
                    self.start_time = time.time()
                elif self.pc.connectionState in ["failed", "closed", "disconnected"]:
                    self.connected = False
            
            # Handle ICE candidates
            @self.pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    await self._send_ice_candidate(candidate)
            
            # Start listening for signaling messages
            await self._handle_signaling()
            
        except Exception as e:
            logger.error(f"Connection error: {e}")
            raise
    
    async def _handle_signaling(self):
        """Handle incoming signaling messages."""
        try:
            async for message in self.websocket:
                data = json.loads(message)
                msg_type = data.get('type')
                
                logger.debug(f"Received signaling message: {msg_type}")
                
                if msg_type == 'offer':
                    await self._handle_offer(data)
                elif msg_type == 'answer':
                    await self._handle_answer(data)
                elif msg_type == 'candidate':
                    await self._handle_ice_candidate(data)
                else:
                    logger.warning(f"Unknown message type: {msg_type}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("WebSocket connection closed")
        except Exception as e:
            logger.error(f"Signaling error: {e}")
    
    async def _handle_offer(self, data: dict):
        """Handle incoming SDP offer."""
        logger.info("Processing offer...")
        
        offer = RTCSessionDescription(sdp=data['sdp'], type='offer')
        await self.pc.setRemoteDescription(offer)
        
        # Create answer
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        
        # Send answer
        await self.websocket.send(json.dumps({
            'type': 'answer',
            'sdp': self.pc.localDescription.sdp
        }))
        logger.info("Answer sent")
    
    async def _handle_answer(self, data: dict):
        """Handle incoming SDP answer."""
        logger.info("Processing answer...")
        answer = RTCSessionDescription(sdp=data['sdp'], type='answer')
        await self.pc.setRemoteDescription(answer)
    
    async def _handle_ice_candidate(self, data: dict):
        """Handle incoming ICE candidate.
        
        Note: aiortc doesn't expose a public RTCIceCandidate constructor.
        It handles ICE candidates internally during SDP negotiation.
        For trickle ICE, we would need lower-level access.
        Since aiortc gathers candidates automatically, remote candidates
        received here are logged but the library manages connectivity internally.
        """
        candidate_str = data.get('candidate', '')
        sdp_mid = data.get('id')
        logger.debug(f"Received ICE candidate for mid={sdp_mid}: {candidate_str[:60] if candidate_str else 'empty'}...")
        # aiortc handles ICE gathering and connectivity internally
        # Remote candidates are applied through the SDP exchange
    
    async def _send_ice_candidate(self, candidate):
        """Send ICE candidate to peer."""
        await self.websocket.send(json.dumps({
            'type': 'candidate',
            'id': candidate.sdpMid,
            'label': candidate.sdpMLineIndex,
            'candidate': candidate.candidate
        }))
    
    async def _handle_video_track(self, track):
        """Process incoming video frames."""
        logger.info("Starting video track processing...")
        
        # Setup video recorder if saving video
        if self.save_video:
            self.recorder = MediaRecorder(self.save_video)
            self.recorder.addTrack(track)
            await self.recorder.start()
            logger.info(f"Recording video to: {self.save_video}")
        
        while True:
            try:
                frame = await track.recv()
                self.frame_count += 1
                
                # Convert to numpy array for OpenCV
                img = frame.to_ndarray(format="bgr24")
                self.latest_frame = img
                
                # Calculate FPS
                if self.start_time and self.frame_count % 30 == 0:
                    elapsed = time.time() - self.start_time
                    fps = self.frame_count / elapsed
                    logger.info(f"Frames: {self.frame_count}, FPS: {fps:.2f}")
                
                # Save individual frames if requested
                if self.save_frames and self.frame_count % 10 == 0:  # Save every 10th frame
                    filename = os.path.join(
                        self.save_frames, 
                        f"frame_{self.frame_count:06d}.jpg"
                    )
                    cv2.imwrite(filename, img)
                    logger.debug(f"Saved frame: {filename}")
                    
                    # Save metadata alongside frame
                    if self.latest_metadata:
                        meta_filename = os.path.join(
                            self.save_frames,
                            f"frame_{self.frame_count:06d}_meta.json"
                        )
                        try:
                            with open(meta_filename, 'w') as f:
                                json.dump(self.latest_metadata, f, indent=2)
                            logger.debug(f"Saved metadata: {meta_filename}")
                        except Exception as e:
                            logger.error(f"Error saving metadata: {e}")
                
                # Display frame if not headless
                if not self.headless:
                    # Add overlay with stats
                    overlay = img.copy()
                    elapsed = time.time() - self.start_time if self.start_time else 0
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    
                    # Basic stats
                    cv2.putText(overlay, f"FPS: {fps:.1f}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Frames: {self.frame_count}", (10, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Resolution: {img.shape[1]}x{img.shape[0]}", (10, 110),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Telemetry overlay from metadata
                    if self.latest_metadata:
                        meta = self.latest_metadata
                        y_offset = 150
                        
                        # Frame number
                        frame_num = meta.get('frameNumber', 'N/A')
                        cv2.putText(overlay, f"Frame: {frame_num}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                        y_offset += 30
                        
                        # GPS data
                        lat = meta.get('latitude', 0)
                        lon = meta.get('longitude', 0)
                        alt_asl = meta.get('altitudeASL', 0)
                        alt_agl = meta.get('altitudeAGL', 0)
                        if lat != 0 or lon != 0:
                            cv2.putText(overlay, f"GPS: {lat:.6f}, {lon:.6f}", (10, y_offset),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            y_offset += 30
                            cv2.putText(overlay, f"Alt: ASL {alt_asl:.1f}m AGL {alt_agl:.1f}m", (10, y_offset),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            y_offset += 30
                        
                        # Gimbal data
                        gimbal_pitch = meta.get('gimbalPitch', 0)
                        gimbal_yaw = meta.get('gimbalYaw', 0)
                        gimbal_roll = meta.get('gimbalRoll', 0)
                        cv2.putText(overlay, f"Gimbal P:{gimbal_pitch:.1f} Y:{gimbal_yaw:.1f} R:{gimbal_roll:.1f}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                        y_offset += 30
                        
                        # Aircraft attitude
                        pitch = meta.get('aircraftPitch', 0)
                        yaw = meta.get('aircraftYaw', 0)
                        roll = meta.get('aircraftRoll', 0)
                        cv2.putText(overlay, f"Attitude P:{pitch:.1f} Y:{yaw:.1f} R:{roll:.1f}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                        y_offset += 30
                        
                        # Velocity data
                        vx = meta.get('velocityX', 0)
                        vy = meta.get('velocityY', 0)
                        vz = meta.get('velocityZ', 0)
                        speed = (vx**2 + vy**2 + vz**2)**0.5
                        cv2.putText(overlay, f"Speed: {speed:.1f} m/s", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                        y_offset += 30
                        
                        # Heading and battery
                        heading = meta.get('aircraftYaw', 0)  # Yaw is heading
                        battery = meta.get('batteryPercent', 0)
                        cv2.putText(overlay, f"Heading: {heading:.1f} deg", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                        y_offset += 30
                        
                        # Battery with color coding
                        battery_color = (0, 255, 0) if battery > 30 else (0, 165, 255) if battery > 15 else (0, 0, 255)
                        cv2.putText(overlay, f"Battery: {battery}%", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, battery_color, 2)
                        y_offset += 30
                        
                        # Satellite count
                        sats = meta.get('satelliteCount', 0)
                        gps_color = (0, 255, 0) if sats > 10 else (0, 165, 255) if sats > 5 else (0, 0, 255)
                        cv2.putText(overlay, f"Sats: {sats}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, gps_color, 2)
                    
                    cv2.imshow("DJI Drone Video Feed", overlay)
                    
                    # Check for quit key
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        logger.info("Quit requested")
                        break
                    elif key == ord('s'):
                        # Save current frame
                        filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                        cv2.imwrite(filename, img)
                        logger.info(f"Saved snapshot: {filename}")
                        
            except Exception as e:
                logger.error(f"Error processing frame: {e}")
                break
        
        # Cleanup
        if self.recorder:
            await self.recorder.stop()
        if not self.headless:
            cv2.destroyAllWindows()
    
    async def close(self):
        """Close all connections."""
        if self.recorder:
            await self.recorder.stop()
        if self.pc:
            await self.pc.close()
        if self.websocket:
            await self.websocket.close()
        if not self.headless:
            cv2.destroyAllWindows()
        logger.info("Connections closed")


async def main():
    parser = argparse.ArgumentParser(description='WebRTC Drone Video Viewer')
    parser.add_argument('--server', '-s', required=True,
                       help='WebSocket server URL (e.g., ws://192.168.1.100:8081)')
    parser.add_argument('--headless', action='store_true',
                       help='Run without displaying video (for servers)')
    parser.add_argument('--save-video', '-v', type=str,
                       help='Save video to file (e.g., output.mp4)')
    parser.add_argument('--save-frames', '-f', type=str,
                       help='Save frames to directory')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    receiver = DroneVideoReceiver(
        server_url=args.server,
        headless=args.headless,
        save_video=args.save_video,
        save_frames=args.save_frames
    )
    
    try:
        await receiver.connect()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await receiver.close()


if __name__ == '__main__':
    print("""
    ╔══════════════════════════════════════════════════════════════╗
    ║           DJI Drone WebRTC Video Viewer                      ║
    ║                                                              ║
    ║  Controls:                                                   ║
    ║    Q - Quit                                                  ║
    ║    S - Save snapshot                                         ║
    ║                                                              ║
    ╚══════════════════════════════════════════════════════════════╝
    """)
    
    asyncio.run(main())
