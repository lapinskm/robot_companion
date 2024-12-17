"""mini-service providing monocular pySlam functionality."""
import json
import os
import uuid

import cv2
from flask import Flask, request, jsonify
from werkzeug.datastructures import FileStorage
from werkzeug.utils import secure_filename
from pyslam.feature_tracker import FeatureTrackerTypes
from pyslam.slam import Slam, Camera

SLAM_SERVICE_DIR = os.path.abspath(os.path.dirname(__file__))
SLAM_CONFIG_DIR = os.path.join(SLAM_SERVICE_DIR, 'config')

# Configuration
CAMERA_JSON_FILE = os.path.join(SLAM_CONFIG_DIR, 'camera.json')
UPLOAD_FOLDER = os.path.join(SLAM_SERVICE_DIR, "uploads")
ALLOWED_EXTENSIONS = {"png", "jpg", "jpeg"}
# Ensure upload directory exists
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
APP_PORT = 8085

# Initialize Flask app
app = Flask(__name__)
app.config["UPLOAD_FOLDER"] = UPLOAD_FOLDER

class SlamSession(Slam):
    """Slam processor instance with session management"""

    @staticmethod
    def load_camera_intrinsics(filename: str = CAMERA_JSON_FILE) -> dict[str]:
        """loads camera intrinsics from JSON file"""
        with open(filename, encoding="utf8") as f:
            return json.load(f)

    def __init__(self) -> None:
        camera_intrinsics = self.load_camera_intrinsics()
        # Initialize monocular SLAM
        camera = Camera(None)
        camera.init_from_json(camera_intrinsics)
        super().__init__(camera=camera, feature_tracker_config={"tracker_type": FeatureTrackerTypes.DES_FLANN})
        self.__last_img_id = -1
        self.__id = str(uuid.uuid4())

    @property
    def id(self) -> str:
        """session id"""
        return self.__id

    def assign_img_id(self) -> int:
        """assigns new image id"""
        self.__last_img_id += 1
        return self.__last_img_id

#Global session storage
slam_sessions: dict[str, SlamSession] = {}

def allowed_file(filename: str) -> bool:
    """Checks if file extension is allowed."""
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

def save_file(file: FileStorage)-> str:
    """Stores uploaded file."""
    filename = secure_filename(file.filename)
    file_path = os.path.join(app.config["UPLOAD_FOLDER"], f"{uuid.uuid4()}_{filename}")
    file.save(file_path)
    return file_path

@app.route("/start_session", methods=["POST"])
def start_session() -> tuple[flask.Response ,int]:
    """Endpoint to start a new SLAM session."""
    slam_session = SlamSession()
    slam_sessions[slam_session.id] = slam_session
    return jsonify({"session_id": slam_session.id}), 200

@app.route("/upload/<session_id>", methods=["POST"])
def upload_image(session_id) -> tuple[flask.Response ,int]:
    """Endpoint to upload an image for SLAM processing."""
    if session_id not in slam_sessions:
        return jsonify({"error": "Invalid session ID"}), 400

    if "file" not in request.files:
        return jsonify({"error": "No file part"}), 400
    file = request.files["file"]

    if file.filename == '':
        return jsonify({"error": "No selected file"}), 400

    if file and allowed_file(file.filename):
        file_path = save_file(file)

        # Process the image with SLAM
        try:
            img = cv2.imread(file_path)
            #monocular case
            slam_sessions[session_id].track(img, img_right=None, depth=None, img_id=None)
        except Exception as e: # noqa
            return jsonify({"error": f"SLAM processing failed: {str(e)}"}), 500

        return jsonify({"message": "Image processed successfully", "file": file.filename}), 200

    return jsonify({"error": "Invalid file type"}), 400

@app.route("/reconstruction/<session_id>", methods=["GET"])
def get_reconstruction(session_id) -> tuple[flask.Response ,int]:
    """Endpoint to get the current 3D reconstruction from SLAM."""
    if session_id not in slam_sessions:
        return jsonify({"error": "Invalid session ID"}), 400
    try:
        reconstruction_data = slam_sessions[session_id].get_dense_map()
        # Serialize reconstruction data to JSON format
        return jsonify({"reconstruction": reconstruction_data}), 200
    except Exception as e:
        return jsonify({"error": f"Could not retrieve reconstruction: {str(e)}"}), 500

@app.route("/end_session/<session_id>", methods=["DELETE"])
def end_session(session_id):
    """Endpoint to end a SLAM session."""
    if session_id not in slam_sessions:
        return jsonify({"error": "Invalid session ID"}), 400

    # Clean up session resources if needed
    del slam_sessions[session_id]
    return jsonify({"message": "Session ended successfully"}), 200

if __name__ == "__main__":
    app.run(debug=True, port=APP_PORT)
