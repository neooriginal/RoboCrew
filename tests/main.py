"""
RoboCrew Web Control - Control your robot via browser with WASD + Mouse
Run: python main.py
Open: http://localhost:5000 (or your robot's IP)
"""

import cv2
import threading
import time
import signal
import sys
from flask import Flask, Response, jsonify, request
from robocrew.robots.XLeRobot.servo_controls import ServoControler

# ============== Configuration ==============
CAMERA_PORT = "/dev/video0"
WHEEL_USB = "/dev/robot_acm0"
HEAD_USB = "/dev/robot_acm1"
WEB_PORT = 5000

# ============== HTML Template ==============
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RoboCrew Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: linear-gradient(135deg, #0d0d1a 0%, #1a1a2e 50%, #16213e 100%);
            min-height: 100vh;
            font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
            color: #e8e8e8;
            overflow: hidden;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
            width: 100%;
            max-width: 1200px;
        }
        .header {
            display: flex;
            align-items: center;
            gap: 15px;
            margin-bottom: 20px;
        }
        h1 {
            font-size: 2.2rem;
            font-weight: 700;
            background: linear-gradient(135deg, #00d4ff 0%, #7b2cbf 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }
        .connection-dot {
            width: 12px; height: 12px;
            border-radius: 50%;
            background: #00ff88;
            box-shadow: 0 0 10px #00ff88;
            animation: pulse 2s infinite;
        }
        .connection-dot.error {
            background: #ff4444;
            box-shadow: 0 0 10px #ff4444;
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.7; transform: scale(1.1); }
        }
        .video-wrapper {
            position: relative;
            width: 100%;
            max-width: 960px;
            border-radius: 20px;
            overflow: hidden;
            box-shadow: 
                0 25px 60px rgba(0, 0, 0, 0.5),
                0 0 100px rgba(0, 212, 255, 0.1),
                inset 0 0 0 1px rgba(255,255,255,0.1);
        }
        .video-container {
            position: relative;
            width: 100%;
            aspect-ratio: 16/9;
            background: #000;
            cursor: pointer;
        }
        .video-container.locked { cursor: none; }
        #video-feed {
            width: 100%;
            height: 100%;
            object-fit: cover;
            display: block;
        }
        .video-overlay {
            position: absolute;
            top: 0; left: 0; right: 0; bottom: 0;
            pointer-events: none;
            background: radial-gradient(ellipse at center, transparent 50%, rgba(0,0,0,0.4) 100%);
        }
        .crosshair {
            position: absolute;
            top: 50%; left: 50%;
            transform: translate(-50%, -50%);
            pointer-events: none;
            opacity: 0;
            transition: opacity 0.3s;
        }
        .video-container.locked .crosshair { opacity: 1; }
        .crosshair-ring {
            width: 50px; height: 50px;
            border: 2px solid rgba(0, 212, 255, 0.7);
            border-radius: 50%;
            position: relative;
        }
        .crosshair-dot {
            position: absolute;
            top: 50%; left: 50%;
            width: 6px; height: 6px;
            background: #00d4ff;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 0 10px #00d4ff;
        }
        .crosshair-line {
            position: absolute;
            background: rgba(0, 212, 255, 0.5);
        }
        .crosshair-line.h { width: 15px; height: 2px; top: 50%; transform: translateY(-50%); }
        .crosshair-line.v { width: 2px; height: 15px; left: 50%; transform: translateX(-50%); }
        .crosshair-line.left { left: -20px; }
        .crosshair-line.right { right: -20px; }
        .crosshair-line.top { top: -20px; }
        .crosshair-line.bottom { bottom: -20px; }
        
        /* Body direction compass */
        .body-compass {
            position: absolute;
            bottom: 20px;
            right: 20px;
            width: 80px;
            height: 80px;
            pointer-events: none;
        }
        .compass-ring {
            width: 100%;
            height: 100%;
            border: 2px solid rgba(255, 170, 0, 0.5);
            border-radius: 50%;
            background: rgba(0, 0, 0, 0.6);
            position: relative;
        }
        .compass-arrow {
            position: absolute;
            top: 50%;
            left: 50%;
            width: 4px;
            height: 30px;
            background: linear-gradient(to top, transparent 0%, #ffaa00 50%, #ff6600 100%);
            transform-origin: center bottom;
            transform: translate(-50%, -100%);
            border-radius: 2px;
            transition: transform 0.1s ease-out;
        }
        .compass-arrow::before {
            content: '';
            position: absolute;
            top: -8px;
            left: 50%;
            transform: translateX(-50%);
            border-left: 6px solid transparent;
            border-right: 6px solid transparent;
            border-bottom: 10px solid #ff6600;
        }
        .compass-center {
            position: absolute;
            top: 50%;
            left: 50%;
            width: 10px;
            height: 10px;
            background: #ffaa00;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 0 8px rgba(255, 170, 0, 0.6);
        }
        .compass-label {
            position: absolute;
            bottom: -22px;
            left: 50%;
            transform: translateX(-50%);
            font-size: 0.7rem;
            color: #ffaa00;
            white-space: nowrap;
            text-shadow: 0 1px 3px rgba(0,0,0,0.8);
        }
        .compass-n {
            position: absolute;
            top: 2px;
            left: 50%;
            transform: translateX(-50%);
            font-size: 0.6rem;
            color: rgba(255,255,255,0.5);
            font-weight: bold;
        }
        
        .click-prompt {
            position: absolute;
            top: 50%; left: 50%;
            transform: translate(-50%, -50%);
            background: rgba(0,0,0,0.8);
            padding: 20px 40px;
            border-radius: 12px;
            text-align: center;
            pointer-events: none;
            transition: opacity 0.3s;
            border: 1px solid rgba(0, 212, 255, 0.3);
        }
        .video-container.locked .click-prompt { opacity: 0; }
        .click-prompt h2 { color: #00d4ff; margin-bottom: 8px; font-size: 1.2rem; }
        .click-prompt p { color: #888; font-size: 0.9rem; }
        
        .controls-panel {
            display: flex;
            gap: 40px;
            margin-top: 25px;
            padding: 20px 40px;
            background: rgba(255,255,255,0.03);
            border-radius: 16px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.08);
        }
        .control-group {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 8px;
        }
        .control-label {
            font-size: 0.7rem;
            color: #666;
            text-transform: uppercase;
            letter-spacing: 2px;
        }
        .control-value {
            font-size: 1.4rem;
            font-weight: 600;
            color: #00d4ff;
            font-family: 'Courier New', monospace;
        }
        .status-indicator {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .status-dot {
            width: 8px; height: 8px;
            border-radius: 50%;
            background: #666;
            transition: all 0.2s;
        }
        .status-dot.active {
            background: #00ff88;
            box-shadow: 0 0 12px #00ff88;
        }
        .status-dot.moving {
            background: #ffaa00;
            box-shadow: 0 0 12px #ffaa00;
        }
        .status-dot.error {
            background: #ff4444;
            box-shadow: 0 0 12px #ff4444;
        }
        
        .keys-display {
            display: grid;
            grid-template-columns: repeat(3, 48px);
            grid-template-rows: repeat(2, 48px);
            gap: 6px;
            margin-top: 20px;
        }
        .key {
            display: flex;
            align-items: center;
            justify-content: center;
            width: 48px; height: 48px;
            background: rgba(255,255,255,0.05);
            border-radius: 10px;
            font-weight: 700;
            font-size: 1.1rem;
            border: 1px solid rgba(255,255,255,0.1);
            transition: all 0.15s;
            color: #888;
        }
        .key:nth-child(1) { grid-column: 2; } /* W */
        .key.active {
            background: rgba(0, 212, 255, 0.25);
            border-color: #00d4ff;
            color: #00d4ff;
            box-shadow: 0 0 20px rgba(0, 212, 255, 0.3);
            transform: scale(0.95);
        }
        .key.empty { visibility: hidden; }
        
        .help-text {
            margin-top: 20px;
            font-size: 0.85rem;
            color: #555;
            text-align: center;
        }
        .help-text kbd {
            background: rgba(255,255,255,0.1);
            padding: 3px 8px;
            border-radius: 4px;
            font-family: inherit;
            border: 1px solid rgba(255,255,255,0.15);
        }
        .debug-panel {
            margin-top: 15px;
            padding: 10px 20px;
            background: rgba(255,0,0,0.1);
            border-radius: 8px;
            font-size: 0.8rem;
            color: #ff8888;
            display: none;
        }
        .debug-panel.show { display: block; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="connection-dot" id="connection-dot"></div>
            <h1>RoboCrew Control</h1>
        </div>
        
        <div class="video-wrapper">
            <div class="video-container" id="video-container">
                <img id="video-feed" src="/video_feed" alt="Robot Camera Feed">
                <div class="video-overlay"></div>
                
                <div class="crosshair">
                    <div class="crosshair-ring">
                        <div class="crosshair-dot"></div>
                    </div>
                    <div class="crosshair-line h left"></div>
                    <div class="crosshair-line h right"></div>
                    <div class="crosshair-line v top"></div>
                    <div class="crosshair-line v bottom"></div>
                </div>
                
                <div class="click-prompt">
                    <h2>🖱️ Click to Control</h2>
                    <p>Mouse controls head • WASD moves robot</p>
                </div>
                
                <!-- Body direction compass -->
                <div class="body-compass">
                    <div class="compass-ring">
                        <span class="compass-n">▲</span>
                        <div class="compass-arrow" id="compass-arrow"></div>
                        <div class="compass-center"></div>
                    </div>
                    <span class="compass-label">Body →</span>
                </div>
            </div>
        </div>
        
        <div class="controls-panel">
            <div class="control-group">
                <span class="control-label">Yaw</span>
                <span class="control-value" id="yaw-value">--</span>
            </div>
            <div class="control-group">
                <span class="control-label">Pitch</span>
                <span class="control-value" id="pitch-value">--</span>
            </div>
            <div class="control-group">
                <span class="control-label">Status</span>
                <div class="status-indicator">
                    <div class="status-dot" id="status-dot"></div>
                    <span class="control-value" id="status-text" style="font-size: 1rem;">Loading...</span>
                </div>
            </div>
        </div>
        
        <div class="keys-display">
            <div class="key" id="key-q">Q</div>
            <div class="key" id="key-w">W</div>
            <div class="key" id="key-e">E</div>
            <div class="key" id="key-a">A</div>
            <div class="key" id="key-s">S</div>
            <div class="key" id="key-d">D</div>
        </div>
        
        <div class="help-text">
            <kbd>ESC</kbd> to release mouse • <kbd>WASD</kbd> to move • Move mouse to look around
        </div>
        
        <div class="debug-panel" id="debug-panel"></div>
    </div>

    <script>
        // DOM Elements
        const videoContainer = document.getElementById('video-container');
        const yawDisplay = document.getElementById('yaw-value');
        const pitchDisplay = document.getElementById('pitch-value');
        const statusDot = document.getElementById('status-dot');
        const statusText = document.getElementById('status-text');
        const connectionDot = document.getElementById('connection-dot');
        const debugPanel = document.getElementById('debug-panel');
        const compassArrow = document.getElementById('compass-arrow');
        
        // State - will be set from server on init
        let mouseLocked = false;
        let currentYaw = null;  // Will be read from robot
        let currentPitch = null;  // Will be read from robot
        let keysPressed = { w: false, a: false, s: false, d: false, q: false, e: false };
        let lastHeadUpdate = 0;
        let headUpdatePending = false;
        let controllerConnected = false;
        let initialized = false;
        let baselineYaw = 0;  // Calibration: yaw value when body is straight
        
        // Settings - optimized for low latency
        const MOUSE_SENS = 0.15;
        const YAW_MIN = -180, YAW_MAX = 180;
        const PITCH_MIN = -180, PITCH_MAX = 180;
        const HEAD_UPDATE_INTERVAL = 33; // ~30 updates/sec for smooth control
        
        function showDebug(msg) {
            debugPanel.textContent = msg;
            debugPanel.classList.add('show');
            console.log('[DEBUG]', msg);
        }
        
        function hideDebug() {
            debugPanel.classList.remove('show');
        }
        
        function updateDisplay() {
            if (currentYaw !== null) {
                yawDisplay.textContent = currentYaw.toFixed(1) + '°';
                updateCompass();
            }
            if (currentPitch !== null) {
                pitchDisplay.textContent = currentPitch.toFixed(1) + '°';
            }
        }
        
        function updateCompass() {
            // Arrow shows where body is facing relative to camera
            // baselineYaw is the yaw when body faces same as camera
            // So relative yaw = currentYaw - baselineYaw
            // Arrow rotation = negative of relative yaw
            const relativeYaw = currentYaw - baselineYaw;
            compassArrow.style.transform = `translate(-50%, -100%) rotate(${-relativeYaw}deg)`;
        }
        
        function updateStatus(text, state) {
            statusText.textContent = text;
            statusDot.className = 'status-dot' + (state ? ' ' + state : '');
        }
        
        // Initialize - read current head position from robot
        async function init() {
            updateStatus('Connecting...', '');
            
            try {
                // Get current head position from robot
                const res = await fetch('/head_position');
                const data = await res.json();
                
                if (data.error) {
                    showDebug('Head position error: ' + data.error);
                    connectionDot.classList.add('error');
                    updateStatus('Error', 'error');
                    return;
                }
                
                // Use actual robot position as starting point
                currentYaw = data.yaw;
                currentPitch = data.pitch;
                baselineYaw = data.yaw;  // This is our "straight ahead" calibration
                controllerConnected = true;
                initialized = true;
                
                updateDisplay();
                connectionDot.classList.remove('error');
                updateStatus('Ready', 'active');
                
                // Show debug info about initial position
                showDebug('Robot position: Yaw=' + currentYaw + ', Pitch=' + currentPitch);
                console.log('Initialized with robot position:', currentYaw, currentPitch);
                
                // Auto-hide debug after 3 seconds
                setTimeout(hideDebug, 3000);
                
            } catch(e) {
                showDebug('Connection error: ' + e.message);
                connectionDot.classList.add('error');
                updateStatus('Offline', 'error');
            }
        }
        
        init();
        
        // Pointer Lock for FPS-style mouse control
        videoContainer.addEventListener('click', () => {
            if (!mouseLocked && initialized) {
                videoContainer.requestPointerLock();
            }
        });
        
        document.addEventListener('pointerlockchange', () => {
            mouseLocked = document.pointerLockElement === videoContainer;
            videoContainer.classList.toggle('locked', mouseLocked);
            if (mouseLocked) {
                updateStatus('Controlling', 'active');
            } else {
                updateStatus('Ready', 'active');
            }
        });
        
        // Throttled head position update with request cancellation
        let headAbortController = null;
        
        function scheduleHeadUpdate() {
            if (!initialized) return;
            
            const now = Date.now();
            const timeSinceLastUpdate = now - lastHeadUpdate;
            
            if (timeSinceLastUpdate >= HEAD_UPDATE_INTERVAL) {
                sendHeadUpdate();
            } else if (!headUpdatePending) {
                headUpdatePending = true;
                setTimeout(() => {
                    headUpdatePending = false;
                    sendHeadUpdate();
                }, HEAD_UPDATE_INTERVAL - timeSinceLastUpdate);
            }
        }
        
        async function sendHeadUpdate() {
            if (currentYaw === null || currentPitch === null) return;
            
            // Cancel any pending request
            if (headAbortController) {
                headAbortController.abort();
            }
            headAbortController = new AbortController();
            
            lastHeadUpdate = Date.now();
            try {
                await fetch('/head', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ yaw: currentYaw, pitch: currentPitch }),
                    signal: headAbortController.signal
                });
                // Don't wait for response parsing - fire and forget for speed
            } catch(e) {
                if (e.name !== 'AbortError') {
                    console.log('Head update error:', e.message);
                }
            }
        }
        
        // Mouse movement handler
        document.addEventListener('mousemove', (e) => {
            if (!mouseLocked || !initialized) return;
            
            // Mouse right = robot looks right (positive yaw change)
            const deltaYaw = e.movementX * MOUSE_SENS;
            const deltaPitch = e.movementY * MOUSE_SENS;
            
            currentYaw = Math.max(YAW_MIN, Math.min(YAW_MAX, currentYaw + deltaYaw));
            currentPitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, currentPitch + deltaPitch));
            
            updateDisplay();
            scheduleHeadUpdate();
        });
        
        // Keyboard controls
        function updateKeyDisplay() {
            ['w', 'a', 's', 'd', 'q', 'e'].forEach(key => {
                const el = document.getElementById('key-' + key);
                if (el) el.classList.toggle('active', keysPressed[key]);
            });
        }
        
        async function sendMovement() {
            const isMoving = Object.values(keysPressed).some(v => v);
            
            if (isMoving) {
                updateStatus('Moving', 'moving');
            } else if (mouseLocked) {
                updateStatus('Controlling', 'active');
            } else {
                updateStatus('Ready', 'active');
            }
            
            try {
                const res = await fetch('/move', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        forward: keysPressed.w,
                        backward: keysPressed.s,
                        left: keysPressed.a,
                        right: keysPressed.d,
                        strafe_left: keysPressed.q,
                        strafe_right: keysPressed.e
                    })
                });
                const data = await res.json();
                if (data.status === 'error') {
                    showDebug('Move error: ' + data.error);
                    updateStatus('Error', 'error');
                }
            } catch(e) {
                showDebug('Move request failed: ' + e.message);
            }
        }
        
        document.addEventListener('keydown', (e) => {
            if (!initialized) return;
            const key = e.key.toLowerCase();
            if (['w', 'a', 's', 'd', 'q', 'e'].includes(key) && !keysPressed[key]) {
                keysPressed[key] = true;
                updateKeyDisplay();
                sendMovement();
            }
        });
        
        document.addEventListener('keyup', (e) => {
            const key = e.key.toLowerCase();
            if (['w', 'a', 's', 'd', 'q', 'e'].includes(key)) {
                keysPressed[key] = false;
                updateKeyDisplay();
                sendMovement();
            }
        });
        
        // Handle window blur - stop movement
        window.addEventListener('blur', () => {
            keysPressed = { w: false, a: false, s: false, d: false, q: false, e: false };
            updateKeyDisplay();
            sendMovement();
        });
    </script>
</body>
</html>
"""

# ============== Flask App ==============
app = Flask(__name__)

# Global state class for thread safety
class RobotState:
    def __init__(self):
        self.camera = None
        self.controller = None
        self.running = True
        self.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        self.lock = threading.Lock()
        self.last_error = None
        # Current head position - read from servos at startup
        self.head_yaw = 0
        self.head_pitch = 0

state = RobotState()


@app.route('/')
def index():
    return HTML_TEMPLATE


@app.route('/status')
def get_status():
    """Get connection status for debugging."""
    return jsonify({
        'controller_connected': state.controller is not None,
        'camera_connected': state.camera is not None and state.camera.isOpened(),
        'head_yaw': state.head_yaw,
        'head_pitch': state.head_pitch,
        'movement': state.movement,
        'error': state.last_error
    })


def generate_frames():
    """MJPEG video stream generator with low latency."""
    while state.running:
        if state.camera is None:
            time.sleep(0.1)
            continue
        try:
            # Flush camera buffer by grabbing frames without decoding
            # This ensures we always get the latest frame
            state.camera.grab()
            state.camera.grab()
            ret, frame = state.camera.retrieve()
            
            if not ret:
                ret, frame = state.camera.read()
                if not ret:
                    time.sleep(0.02)
                    continue
            
            # Lower quality = faster encoding = lower latency
            _, buffer = cv2.imencode('.jpg', frame, [
                cv2.IMWRITE_JPEG_QUALITY, 50,
                cv2.IMWRITE_JPEG_OPTIMIZE, 0
            ])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except Exception as e:
            state.last_error = f"Camera error: {str(e)}"
            time.sleep(0.05)


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/head_position')
def get_head_position():
    """Get current head servo positions - read fresh from servos."""
    if state.controller is None:
        return jsonify({'error': 'No controller connected'})
    try:
        pos = state.controller.get_head_position()
        print(f"[HEAD READ] Raw position from servos: {pos}")
        yaw = round(pos.get(7, 0), 1)
        pitch = round(pos.get(8, 0), 1)
        print(f"[HEAD READ] Parsed: yaw={yaw}, pitch={pitch}")
        # Update our cached values
        state.head_yaw = yaw
        state.head_pitch = pitch
        return jsonify({'yaw': yaw, 'pitch': pitch})
    except Exception as e:
        state.last_error = f"Head read error: {str(e)}"
        print(f"[HEAD READ ERROR] {e}")
        return jsonify({'error': str(e)})


@app.route('/head', methods=['POST'])
def set_head():
    """Set head yaw and pitch - smooth incremental control."""
    if state.controller is None:
        return jsonify({'status': 'error', 'error': 'No controller connected'})
    
    data = request.json
    yaw = float(data.get('yaw', state.head_yaw))
    pitch = float(data.get('pitch', state.head_pitch))
    
    print(f"[HEAD WRITE] Commanding: yaw={yaw}, pitch={pitch}")
    
    try:
        state.controller.turn_head_yaw(yaw)
        state.controller.turn_head_pitch(pitch)
        state.head_yaw = yaw
        state.head_pitch = pitch
        return jsonify({'status': 'ok', 'yaw': yaw, 'pitch': pitch})
    except Exception as e:
        state.last_error = f"Head write error: {str(e)}"
        print(f"[HEAD WRITE ERROR] {e}")
        return jsonify({'status': 'error', 'error': str(e)})


@app.route('/move', methods=['POST'])
def move():
    """Update movement state from WASD+QE keys."""
    if state.controller is None:
        return jsonify({'status': 'error', 'error': 'No controller connected'})
    
    data = request.json
    with state.lock:
        state.movement = {
            'forward': bool(data.get('forward')),
            'backward': bool(data.get('backward')),
            'left': bool(data.get('left')),
            'right': bool(data.get('right')),
            'strafe_left': bool(data.get('strafe_left')),
            'strafe_right': bool(data.get('strafe_right'))
        }
    
    # Send command immediately for responsiveness
    try:
        if state.movement['forward']:
            state.controller._wheels_write('up')
        elif state.movement['backward']:
            state.controller._wheels_write('down')
        elif state.movement['strafe_left']:
            state.controller._wheels_write('strafe_left')
        elif state.movement['strafe_right']:
            state.controller._wheels_write('strafe_right')
        elif state.movement['left']:
            state.controller._wheels_write('left')
        elif state.movement['right']:
            state.controller._wheels_write('right')
        else:
            state.controller._wheels_stop()
        return jsonify({'status': 'ok'})
    except Exception as e:
        state.last_error = f"Movement error: {str(e)}"
        return jsonify({'status': 'error', 'error': str(e)})


def movement_loop():
    """Continuous movement control thread - keeps wheels moving while key held."""
    while state.running:
        if state.controller is None:
            time.sleep(0.1)
            continue
        
        with state.lock:
            movement = state.movement.copy()
        
        try:
            if movement.get('forward'):
                state.controller._wheels_write('up')
            elif movement.get('backward'):
                state.controller._wheels_write('down')
            elif movement.get('strafe_left'):
                state.controller._wheels_write('strafe_left')
            elif movement.get('strafe_right'):
                state.controller._wheels_write('strafe_right')
            elif movement.get('left'):
                state.controller._wheels_write('left')
            elif movement.get('right'):
                state.controller._wheels_write('right')
            else:
                state.controller._wheels_stop()
        except Exception as e:
            state.last_error = f"Movement loop error: {str(e)}"
        
        time.sleep(0.05)


def cleanup(signum=None, frame=None):
    """Graceful shutdown."""
    print("\n🛑 Shutting down...")
    state.running = False
    
    if state.controller:
        try:
            state.controller._wheels_stop()
            state.controller.disconnect()
            print("✓ Controller disconnected")
        except Exception as e:
            print(f"✗ Controller cleanup error: {e}")
    
    if state.camera:
        try:
            state.camera.release()
            print("✓ Camera released")
        except Exception as e:
            print(f"✗ Camera cleanup error: {e}")
    
    sys.exit(0)


# ============== Main Entry Point ==============
if __name__ == "__main__":
    print("=" * 50)
    print("🤖 RoboCrew Web Control")
    print("=" * 50)
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    # Initialize camera
    print(f"📷 Connecting camera ({CAMERA_PORT})...", end=" ", flush=True)
    try:
        state.camera = cv2.VideoCapture(CAMERA_PORT)
        state.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        state.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        state.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if state.camera.isOpened():
            print("✓")
        else:
            print("⚠ Warning: Camera may not be available")
            state.last_error = "Camera not available"
    except Exception as e:
        print(f"✗ Failed: {e}")
        state.camera = None
        state.last_error = f"Camera init failed: {e}"
    
    # Initialize servo controller - DO NOT reset position!
    print(f"🔧 Connecting servos ({WHEEL_USB}, {HEAD_USB})...", end=" ", flush=True)
    try:
        state.controller = ServoControler(WHEEL_USB, HEAD_USB)
        print("✓")
        
        # Read current head position (don't move it!)
        print("� Reading current head position...", end=" ", flush=True)
        try:
            pos = state.controller.get_head_position()
            state.head_yaw = round(pos.get(7, 0), 1)
            state.head_pitch = round(pos.get(8, 0), 1)
            print(f"✓ (Yaw: {state.head_yaw}°, Pitch: {state.head_pitch}°)")
        except Exception as e:
            print(f"⚠ Could not read: {e}")
            state.head_yaw = 0
            state.head_pitch = 35
        
    except Exception as e:
        print(f"✗ Failed: {e}")
        state.controller = None
        state.last_error = f"Controller init failed: {e}"
    
    # Start movement control thread
    print("🔄 Starting movement thread...", end=" ", flush=True)
    movement_thread = threading.Thread(target=movement_loop, daemon=True)
    movement_thread.start()
    print("✓")
    
    # Start web server
    print()
    print(f"🌐 Web server starting on http://0.0.0.0:{WEB_PORT}")
    print(f"   Open in browser to control the robot!")
    print()
    if state.last_error:
        print(f"⚠ Last error: {state.last_error}")
    print("Press Ctrl+C to stop")
    print("-" * 50)
    
    try:
        app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, use_reloader=False, debug=False)
    except KeyboardInterrupt:
        cleanup()
