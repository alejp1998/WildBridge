#!/usr/bin/env python3
"""
Debug script to verify metadata reception from WebRTC data channels.

This script connects to the WebRTC server and logs all messages received
from data channels to help diagnose metadata transmission issues.

Usage:
    python debug_metadata.py --server ws://192.168.1.100:8081
"""

import argparse
import asyncio
import json
import logging
import sys

import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class MetadataDebugger:
    def __init__(self, server_url: str):
        self.server_url = server_url
        self.pc = None
        self.websocket = None
        self.client_id = None
        self.metadata_received = 0
        self.metadata_channel = None

    async def connect(self):
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
            logger.info("Registered as viewer")
            
            # Create peer connection
            self.pc = RTCPeerConnection()
            logger.info("Created RTCPeerConnection")
            
            # Create negotiated data channel for metadata
            try:
                self.metadata_channel = self.pc.createDataChannel(
                    "telemetry",
                    negotiated=True,
                    id=0,
                    ordered=True
                )
                logger.info(f"Created metadata channel (negotiated): {self.metadata_channel.label}")
                
                @self.metadata_channel.on("open")
                def on_open():
                    logger.info("Metadata channel OPENED")
                
                @self.metadata_channel.on("close")
                def on_close():
                    logger.info("Metadata channel CLOSED")
                
                @self.metadata_channel.on("message")
                def on_message(message):
                    self.metadata_received += 1
                    logger.info(f"=== METADATA RECEIVED #{self.metadata_received} ===")
                    try:
                        meta = json.loads(message)
                        logger.info(f"Frame: {meta.get('frameNumber')}")
                        logger.info(f"GPS: {meta.get('latitude'):.6f}, {meta.get('longitude'):.6f}")
                        logger.info(f"Alt: {meta.get('altitudeAGL'):.1f}m")
                        logger.info(f"Battery: {meta.get('batteryPercent')}%")
                        logger.info(f"Full metadata: {json.dumps(meta, indent=2)}")
                    except Exception as e:
                        logger.error(f"Error parsing metadata: {e}")
                        logger.error(f"Raw message: {message[:200]}")
                
            except Exception as e:
                logger.error(f"Error creating metadata channel: {e}")
            
            # Handle incoming data channels from peer
            @self.pc.on("datachannel")
            def on_datachannel(channel):
                logger.info(f"=== RECEIVED DATA CHANNEL: {channel.label} ===")
                
                if channel.label == "telemetry":
                    logger.info("This is the telemetry channel from peer!")
                    
                    @channel.on("open")
                    def on_open():
                        logger.info(f"Peer's telemetry channel OPENED")
                    
                    @channel.on("message")
                    def on_message(message):
                        self.metadata_received += 1
                        logger.info(f"=== PEER METADATA RECEIVED #{self.metadata_received} ===")
                        try:
                            meta = json.loads(message)
                            logger.info(f"Frame: {meta.get('frameNumber')}")
                            logger.info(f"GPS: {meta.get('latitude'):.6f}, {meta.get('longitude'):.6f}")
                            logger.info(f"Alt: {meta.get('altitudeAGL'):.1f}m")
                            logger.info(f"Battery: {meta.get('batteryPercent')}%")
                        except Exception as e:
                            logger.error(f"Error parsing metadata: {e}")
                            logger.error(f"Raw message: {message[:200]}")
            
            # Handle ICE candidates
            @self.pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    logger.debug(f"Sending ICE candidate: {candidate.candidate[:50] if candidate.candidate else 'empty'}")
                    await self.websocket.send(json.dumps({
                        'type': 'candidate',
                        'id': candidate.sdpMid,
                        'label': candidate.sdpMLineIndex,
                        'candidate': candidate.candidate
                    }))
            
            # Handle connection state changes
            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                logger.info(f"Connection state: {self.pc.connectionState}")
            
            # Handle signaling messages
            await self._handle_signaling()
            
        except Exception as e:
            logger.error(f"Connection error: {e}")
            raise

    async def _handle_signaling(self):
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
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("WebSocket connection closed")
        except Exception as e:
            logger.error(f"Signaling error: {e}")

    async def _handle_offer(self, data: dict):
        logger.info("Processing offer...")
        offer = RTCSessionDescription(sdp=data['sdp'], type='offer')
        await self.pc.setRemoteDescription(offer)
        
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        
        await self.websocket.send(json.dumps({
            'type': 'answer',
            'sdp': self.pc.localDescription.sdp
        }))
        logger.info("Answer sent")

    async def _handle_answer(self, data: dict):
        logger.info("Processing answer...")
        answer = RTCSessionDescription(sdp=data['sdp'], type='answer')
        await self.pc.setRemoteDescription(answer)

    async def _handle_ice_candidate(self, data: dict):
        logger.debug(f"Received ICE candidate: {data.get('candidate', '')[:50]}")

    async def close(self):
        if self.pc:
            await self.pc.close()
        if self.websocket:
            await self.websocket.close()
        logger.info(f"Total metadata received: {self.metadata_received}")


async def main():
    parser = argparse.ArgumentParser(description='WebRTC Metadata Debug Tool')
    parser.add_argument('--server', '-s', required=True,
                       help='WebSocket server URL (e.g., ws://192.168.1.100:8081)')
    
    args = parser.parse_args()
    
    debugger = MetadataDebugger(args.server)
    
    try:
        await debugger.connect()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await debugger.close()


if __name__ == '__main__':
    asyncio.run(main())
