import socket
import threading
import time
import io
import serial
import cv2
from flask import Flask, Response

from flask import Flask, Response
from picamera2 import Picamera2
from gpiozero import Robot 

# Network Configuration
VIDEO_STREAM_HOST = '0.0.0.0'
VIDEO_STREAM_PORT = 5000
MOTOR_SERVER_HOST = '0.0.0.0'
MOTOR_SERVER_PORT = 65432 

maxForwardSpeed = 100

try:
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    
    # Let the port settle
    ser.flush()
    print("Serial port opened successfully.")
except serial.SerialException as e:
    print(f"Error opening or writing to serial port: {e}")

def execute_command(command):
    """Parses and executes a motor command."""
    print(f"[Motor Thread] Received command: {command}")
    command, *_ = command.split("n")
    print(f'cleaned cmd {command}')
    try:
        if command == 'stop':
            print("stopping")
            leftMotorsSpeed = 0
            rightMotorsSpeed = 0
            try:
                ser.write(('m 0 0 0 0\n').encode('utf-8'))
            except serial.SerialException as e:
                print(f"Error writing to serial port: {e}")
        else:
            leftMotorsSpeed, rightMotorsSpeed = map(float, command.split(' '))
            leftMotorsSpeed*=maxForwardSpeed
            rightMotorsSpeed*=maxForwardSpeed
            
            leftMotorsSpeed += 5
            rightMotorsSpeed += 5
            
            #print(f'lmspeed {leftMotorsSpeed} rmspeed {rightMotorsSpeed}')
            motorVels = f'm {int(leftMotorsSpeed)} {int(rightMotorsSpeed)} {int(leftMotorsSpeed)} {int(rightMotorsSpeed)}\n'
            if leftMotorsSpeed > rightMotorsSpeed:
                print("going right")
            else:
                print("going left")
            print(f'motor vels {motorVels}')
            try:
                ser.write(motorVels.encode('utf-8'))
            except serial.SerialException as e:
                print(f"Error writing to serial port: {e}")
    except:
        print(f"Invalid command format {command}")

def start_motor_server():
    """This function runs in a separate thread to handle motor commands."""
    print("[Motor Thread] Starting motor control server...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((MOTOR_SERVER_HOST, MOTOR_SERVER_PORT))
        s.listen()
        print(f"[Motor Thread] Listening for motor commands on port {MOTOR_SERVER_PORT}...")
        
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"[Motor Thread] Connected by {addr}")
                try:
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            print(f"[Motor Thread] Client {addr} disconnected.")
                            execute_command('stop') # Safety stop
                            break
                        command = data.decode('utf-8')
                        execute_command(command)
                except ConnectionResetError:
                    print(f"[Motor Thread] Connection with {addr} was forcibly closed.")
                    execute_command('stop')
                except Exception as e:
                    print(f"[Motor Thread] An error occurred: {e}")
                    execute_command('stop')

# ==============================================================================
# --- VIDEO STREAMING SERVER (Flask-based) ---
# ==============================================================================

raspicam = False
if raspicam:
    app = Flask(__name__)

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        exit()
    def generate_frames():
        """Generator function that yields camera frames."""
        with Picamera2() as camera:
            camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))
            camera.start()
            time.sleep(2)  # Allow camera to warm up

            while True:
                buffer = io.BytesIO()
                camera.capture_file(buffer, format='jpeg')
                frame = buffer.getvalue()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


    @app.route('/video_feed')
    def video_feed():
        """Video streaming route."""
        return Response(generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

else:
    app = Flask(__name__)
    cap = cv2.VideoCapture(0) # Use camera index 0

    def generate_frames():
        """Generator function that yields camera frames."""
        if not cap.isOpened():
            print("Error: Could not open video stream.")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Stream ended or camera disconnected.")
                break

            # Encode the frame as JPEG
            is_success, encoded_image = cv2.imencode(".jpg", frame)
            if is_success:
                # FIX: Convert the numpy array of bytes to a standard bytes object
                frame_bytes = encoded_image.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    @app.route('/video_feed')
    def video_feed():
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
# ==============================================================================
# --- MAIN EXECUTION BLOCK ---
# ==============================================================================

if __name__ == '__main__':
    # Create and start the motor server thread
    # Setting daemon=True means the thread will exit when the main program exits
    motor_thread = threading.Thread(target=start_motor_server, daemon=True)
    motor_thread.start()
    
    # Start the Flask web server in the main thread
    # The 'use_reloader=False' is important to prevent issues with threading.
    print("[Main Thread] Starting Flask video stream server...")
    app.run(host=VIDEO_STREAM_HOST, port=VIDEO_STREAM_PORT, threaded=True, use_reloader=False)






while false:   
    from flask import Flask, Response
    from picamera2 import Picamera2
    import io
    import time

    app = Flask(__name__)

    def generate_frames():
        with Picamera2() as camera:
            camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))
            camera.start()
            time.sleep(2)  # Allow the camera to warm up

            while True:
                buffer = io.BytesIO()
                camera.capture_file(buffer, format='jpeg')
                frame = buffer.getvalue()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    @app.route('/video_feed')
    def video_feed():
        return Response(generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    if __name__ == '__main__':
        app.run(host='0.0.0.0', port=5000, threaded=True)
