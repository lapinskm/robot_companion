"""Simple chessboard camera calibration util generating json files compatible with pySlam API"""

import argparse
import json
import numpy as np
import cv2

def create_calibration_dict(mtx: np.ndarray, dist: np.ndarray, frame_shape: np.ndarray) -> dict:
    """Converts opencv calibration parameters to PySlam camera data structure."""
    return {
        "type": 0,  # Replace with appropriate CameraTypes enum value
        "width": frame_shape[1],
        "height": frame_shape[0],
        "fx": mtx[0, 0],
        "fy": mtx[1, 1],
        "cx": mtx[0, 2],
        "cy": mtx[1, 2],
        "D": json.dumps(dist.flatten().tolist()),
        "fps": 30,  # Example value, replace as needed
        "bf": 0.0,  # Example value, replace as needed
        "b": 0.0,  # Example value, replace as needed
        "depth_factor": 1.0,  # Example value, replace as needed
        "depth_threshold": 0.0,  # Example value, replace as needed
        "is_distorted": np.any(dist) is True,
        "u_min": 0.0,
        "u_max": frame_shape[1],
        "v_min": 0.0,
        "v_max": frame_shape[0],
        "initialized": True
    }

def calibrate_camera(image_paths: list[str], chessboard_size: tuple[int, int],
                     square_size: float, show_chessboards:bool) -> dict:
    """
    Perform camera calibration using chessboard images.

    Args:
        image_paths (list): List of paths to chessboard images.
        chessboard_size (tuple): Number of inner corners per chessboard row and column (rows, cols).
        square_size (float): Size of a square in the chessboard (e.g., in meters or centimeters).

    Returns:
        dict: Calibration parameters in a dictionary format.
    """
    # Termination criteria for corner refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points for the chessboard pattern
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[1], 0:chessboard_size[0]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    for image_path in image_paths:
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            objpoints.append(objp)

            if ret and show_chessboards:
                cv2.namedWindow("corners", cv2.WINDOW_NORMAL)
                fnl = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
                cv2.imshow("corners", fnl)
                cv2.waitKey(500)

            # Refine corner locations
            refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(refined_corners)
        else:
            print(f"Chessboard not found in {image_path}")

    # Perform camera calibration
    ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],
                                               cameraMatrix=None, distCoeffs=None)

    if not ret:
        raise RuntimeError("Camera calibration failed")


    return create_calibration_dict(mtx, dist, gray.shape)

def main() -> None:
    """Main function."""
    parser = argparse.ArgumentParser(description="Chessboard-based camera calibration.")
    parser.add_argument("--images", nargs='+', required=True, help="Paths to chessboard images.")
    parser.add_argument("--output", required=True, help="Output JSON file path.")
    parser.add_argument("--rows",
                        type=int, required=True, help="Number of inner corners per chessboard row.")
    parser.add_argument("--cols",
                        type=int, required=True, help="Number of inner corners per chessboard column.")
    parser.add_argument("--square_size",
                        type=float, required=True, help="Size of a square in the chessboard.")
    parser.add_argument("--show_chessboards", required=False,  action="store_true",
                        help="Show detected corners of the chessboard.")

    args = parser.parse_args()

    chessboard_size = (args.rows, args.cols)
    square_size = args.square_size

    calibration_data = calibrate_camera(args.images, chessboard_size, square_size, args.show_chessboards)
    with open(args.output, 'w', encoding="utf8") as f:
        json.dump(calibration_data, f, indent=2)
    print(f"Calibration data saved to {args.output}")

if __name__ == "__main__":
    main()
