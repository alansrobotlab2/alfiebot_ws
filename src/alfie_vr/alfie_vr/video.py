import asyncio
import base64
import cv2
import socketio
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Initialize FastAPI and Socket.IO
app = FastAPI()
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
socket_app = socketio.ASGIApp(sio, app)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Video State
is_streaming = False
stream_task = None

async def video_stream_loop():
    global is_streaming
    print("Starting video capture loop...")
    
    # Open camera - try index 0 (/dev/video0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        # Try index 2 if 0 fails (sometimes 0/1 are metadata)
        print("Could not open video device 0, trying 2...")
        cap = cv2.VideoCapture(2)
        if not cap.isOpened():
            print("Error: Could not open any video device")
            is_streaming = False
            return

    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print(f"Camera opened: {cap.getBackendName()}")

    try:
        while is_streaming:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                await asyncio.sleep(0.1)
                continue

            # Encode to JPEG
            # Quality 70 is a good balance for streaming
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            frame_b64 = base64.b64encode(buffer).decode('utf-8')

            # Emit to all connected clients
            await sio.emit('video_frame', {'frame': frame_b64})
            
            # Yield control to event loop
            await asyncio.sleep(0.01)
            
    except Exception as e:
        print(f"Stream error: {e}")
    finally:
        cap.release()
        print("Camera released.")
        is_streaming = False

@sio.event
async def connect(sid, environ):
    print(f"Client connected: {sid}")

@sio.event
async def disconnect(sid):
    print(f"Client disconnected: {sid}")

@sio.event
async def start_video_stream(sid):
    global is_streaming, stream_task
    print(f"Client {sid} requested video stream")
    
    if not is_streaming:
        is_streaming = True
        stream_task = asyncio.create_task(video_stream_loop())

@sio.event
async def stop_video_stream(sid):
    # For this simple server, we don't stop the stream on individual stop requests
    # to keep it simple for multiple viewers, or we could just log it.
    print(f"Client {sid} requested stop video stream")

if __name__ == "__main__":
    print("Starting Simple Video Server on port 8000...")
    print("Ensure no other service is using port 8000")
    
    # Check for SSL certs
    import os
    ssl_keyfile = "key.pem"
    ssl_certfile = "cert.pem"
    
    if os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile):
        print(f"Using SSL certificates: {ssl_certfile}, {ssl_keyfile}")
        uvicorn.run(socket_app, host="0.0.0.0", port=8000, ssl_keyfile=ssl_keyfile, ssl_certfile=ssl_certfile)
    else:
        print("Warning: SSL certificates not found. Running in HTTP mode (might be blocked by browser if main page is HTTPS)")
        uvicorn.run(socket_app, host="0.0.0.0", port=8000)
