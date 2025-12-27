from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import threading
import time

app = Flask(__name__)

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"size": (1280, 720)}
))
picam2.start()

frame = None
lock = threading.Lock()

def capture_frames():
    global frame
    while True:
        img = picam2.capture_array()
        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with lock:
            frame = jpeg.tobytes()
        time.sleep(0.03)  # ~30 FPS

threading.Thread(target=capture_frames, daemon=True).start()

@app.route('/')
def index():
    return "Camera stream at /stream"

@app.route('/stream')
def stream():
    def generate():
        while True:
            with lock:
                if frame is None:
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)

    return Response(generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, threaded=True)
