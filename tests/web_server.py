"""Flask web server for robot control with WASD and mouse controls."""

import cv2
import threading
from flask import Flask, render_template, Response, request, jsonify


class RobotWebServer:
    """Web server for controlling robot via browser interface."""
    
    def __init__(self, servo_controller, camera, host='0.0.0.0', port=5000):
        """
        Initialize the web server.
        
        Args:
            servo_controller: ServoControler instance for robot movement
            camera: OpenCV VideoCapture instance for video streaming
            host: Host address to bind to (default: 0.0.0.0)
            port: Port to run the server on (default: 5000)
        """
        self.servo_controller = servo_controller
        self.camera = camera
        self.host = host
        self.port = port
        self.app = Flask(__name__)
        self._setup_routes()
        
        # Current head position tracking
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        
    def _setup_routes(self):
        """Set up Flask routes."""
        
        @self.app.route('/')
        def index():
            """Serve the main control interface."""
            return render_template('index.html')
        
        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route."""
            return Response(
                self._generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/control', methods=['POST'])
        def control():
            """Handle movement control commands."""
            data = request.json
            action = data.get('action')
            
            try:
                if action == 'forward':
                    distance = data.get('distance', 0.1)  # Default 0.1 meters
                    self.servo_controller.go_forward(distance)
                    return jsonify({'status': 'success', 'message': f'Moving forward {distance}m'})
                
                elif action == 'backward':
                    distance = data.get('distance', 0.1)
                    self.servo_controller.go_backward(distance)
                    return jsonify({'status': 'success', 'message': f'Moving backward {distance}m'})
                
                elif action == 'turn_left':
                    angle = data.get('angle', 15)  # Default 15 degrees
                    self.servo_controller.turn_left(angle)
                    return jsonify({'status': 'success', 'message': f'Turning left {angle}°'})
                
                elif action == 'turn_right':
                    angle = data.get('angle', 15)
                    self.servo_controller.turn_right(angle)
                    return jsonify({'status': 'success', 'message': f'Turning right {angle}°'})
                
                else:
                    return jsonify({'status': 'error', 'message': 'Unknown action'}), 400
                    
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @self.app.route('/look', methods=['POST'])
        def look():
            """Handle head/camera look commands."""
            data = request.json
            
            try:
                # Get delta movements from mouse
                delta_yaw = data.get('deltaYaw', 0)
                delta_pitch = data.get('deltaPitch', 0)
                
                # Update current positions
                self.current_yaw += delta_yaw
                self.current_pitch += delta_pitch
                
                # Clamp values to reasonable ranges
                # Yaw: -180 to 180 degrees
                self.current_yaw = max(-180, min(180, self.current_yaw))
                # Pitch: -90 to 90 degrees  
                self.current_pitch = max(-90, min(90, self.current_pitch))
                
                # Send commands to servos
                if abs(delta_yaw) > 0.1:  # Only update if significant movement
                    self.servo_controller.turn_head_yaw(self.current_yaw)
                
                if abs(delta_pitch) > 0.1:
                    self.servo_controller.turn_head_pitch(self.current_pitch)
                
                return jsonify({
                    'status': 'success',
                    'yaw': self.current_yaw,
                    'pitch': self.current_pitch
                })
                
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500
    
    def _generate_frames(self):
        """Generate video frames for streaming."""
        while True:
            success, frame = self.camera.read()
            if not success:
                break
            else:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                
                # Yield frame in multipart format
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    
    def run(self, debug=False):
        """Run the Flask server."""
        self.app.run(host=self.host, port=self.port, debug=debug, threaded=True)
    
    def run_in_thread(self):
        """Run the Flask server in a separate thread."""
        thread = threading.Thread(target=self.run, kwargs={'debug': False})
        thread.daemon = True
        thread.start()
        return thread
