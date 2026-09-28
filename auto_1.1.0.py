import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import time
import math
import threading
from inference_sdk import InferenceHTTPClient, InferenceConfiguration

# ==========================================
#               CONFIGURATION
# ==========================================
ESP32_IP = "10.218.230.31"  # Replace with actual IP shown on TFT screen
UDP_PORT = 4210
CAMERA_INDEX = 0

WARP_W = 1000
WARP_H = 750
MM_PER_PIXEL = 2.0
PROXIMITY_THRESHOLD_MM = 150  # 15cm trigger distance

# Roboflow Configuration
RF_API_KEY = "X5wKaATp2EknpnzoIAqF"
RF_WORKSPACE = "prem-supthaksina"
RF_WORKFLOW = "arena-gem-and-drop-zone-detectio"
RF_QUERY_W = 640   # Downscale width for faster Roboflow upload
RF_QUERY_H = 480   # Downscale height for faster Roboflow upload

# ==========================================
#               GLOBALS
# ==========================================
calibration_points = []
homography_matrix = None

# Lock-free frame sharing via simple reference swap
_latest_frame_for_rf = None
_rf_frame_lock = threading.Lock()

roboflow_target = None
roboflow_predictions = []
roboflow_active = True

# UDP Socket (non-blocking)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

# ==========================================
#          THREADED CAMERA CAPTURE
# ==========================================
class FastCapture:
    """Dedicated thread that always holds the latest frame, so cap.read() never blocks the main loop."""
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize internal buffer lag
        self.ret = False
        self.frame = None
        self._lock = threading.Lock()
        self._stopped = False
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while not self._stopped:
            ret, frame = self.cap.read()
            with self._lock:
                self.ret, self.frame = ret, frame

    def read(self):
        with self._lock:
            return self.ret, self.frame

    def release(self):
        self._stopped = True
        self.cap.release()

# ==========================================
#               HELPERS
# ==========================================
def send_udp(cmd):
    try:
        sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
    except Exception:
        pass

def extract_roboflow_predictions(data):
    """
    Recursively scans the Roboflow Workflow JSON response to find
    all bounding boxes and returns them as a list of dictionaries.
    """
    preds = []
    if isinstance(data, dict):
        if 'x' in data and 'y' in data and 'width' in data and 'height' in data:
            preds.append(data)
        elif 'center_x' in data and 'center_y' in data and 'width' in data and 'height' in data:
            data_copy = dict(data)
            data_copy['x'] = data['center_x']
            data_copy['y'] = data['center_y']
            preds.append(data_copy)

        for k, v in data.items():
            preds.extend(extract_roboflow_predictions(v))

    elif isinstance(data, list):
        for item in data:
            preds.extend(extract_roboflow_predictions(item))

    return preds

def get_color_for_class(class_name):
    """Returns a specific BGR color based on class name string."""
    name = class_name.lower()
    if 'red' in name: return (50, 50, 220)
    if 'blue' in name: return (255, 150, 50)
    if 'green' in name: return (50, 220, 50)
    if 'purple' in name: return (180, 50, 130)
    if 'cyan' in name: return (255, 220, 50)
    if 'orange' in name: return (50, 140, 255)

    h = hash(class_name)
    return ((h & 0xFF), ((h >> 8) & 0xFF), ((h >> 16) & 0xFF))

def scale_predictions_back(preds, sx, sy):
    """Scale bounding box coordinates from the downscaled query image back to the full warp size."""
    scaled = []
    for p in preds:
        sp = dict(p)
        sp['x'] = p['x'] * sx
        sp['y'] = p['y'] * sy
        sp['width'] = p['width'] * sx
        sp['height'] = p['height'] * sy
        scaled.append(sp)
    return scaled

# ==========================================
#           ROBOFLOW BACKGROUND THREAD
# ==========================================
def roboflow_thread():
    global roboflow_target, roboflow_predictions
    client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",
        api_key=RF_API_KEY
    ).configure(InferenceConfiguration(api_key_transport="header"))

    print("[Roboflow Thread] Started!")
    while roboflow_active:
        # Grab the latest frame snapshot
        with _rf_frame_lock:
            frame_snap = _latest_frame_for_rf

        if frame_snap is not None:
            try:
                result = client.run_workflow(
                    workspace_name=RF_WORKSPACE,
                    workflow_id=RF_WORKFLOW,
                    images={"image": frame_snap},
                    use_cache=True
                )

                preds = extract_roboflow_predictions(result)

                # Scale coordinates back to full warp resolution
                sx = WARP_W / RF_QUERY_W
                sy = WARP_H / RF_QUERY_H
                preds = scale_predictions_back(preds, sx, sy)

                roboflow_predictions = preds

                if preds:
                    roboflow_target = (int(preds[0]['x']), int(preds[0]['y']))
                else:
                    roboflow_target = None

            except Exception as e:
                print(f"[Roboflow Thread] Error: {e}")

        # No sleep — fire again as soon as the API returns.
        # The network round-trip IS the throttle (~200-400ms).
        time.sleep(0.05)  # Tiny yield to prevent CPU spin

def order_points(pts):
    """
    Sorts 4 points in the order: Top-Left, Top-Right, Bottom-Right, Bottom-Left.
    Prevents black screens caused by clicking points in the wrong order (bow-tie warp).
    """
    xSorted = pts[np.argsort(pts[:, 0]), :]
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost
    rightMost = rightMost[np.argsort(rightMost[:, 1]), :]
    (tr, br) = rightMost
    return np.array([tl, tr, br, bl], dtype="float32")

# ==========================================
#           MOUSE CALLBACK
# ==========================================
def mouse_callback(event, x, y, flags, param):
    global calibration_points, homography_matrix
    if event == cv2.EVENT_LBUTTONDOWN and len(calibration_points) < 4:
        calibration_points.append((x, y))
        print(f"Calibration Point {len(calibration_points)}: ({x}, {y})")
        if len(calibration_points) == 4:
            pts_src = np.array(calibration_points, dtype=np.float32)
            pts_src = order_points(pts_src)

            pts_dst = np.array([
                [0, 0],
                [WARP_W, 0],
                [WARP_W, WARP_H],
                [0, WARP_H]
            ], dtype=np.float32)
            homography_matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
            print(">>> Homography Matrix Computed! Switching to AI Navigation... <<<")

# ==========================================
#               MAIN
# ==========================================
def main():
    global _latest_frame_for_rf, roboflow_active, homography_matrix, calibration_points

    cap = FastCapture(CAMERA_INDEX)

    cv2.namedWindow("Arena")
    cv2.setMouseCallback("Arena", mouse_callback)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    threading.Thread(target=roboflow_thread, daemon=True).start()

    print("\n--- INSTRUCTIONS ---")
    print("1. Click the 4 corners in any order (auto-sorted).")
    print("2. The system will auto-warp to 1000x750 and start steering the robot.")
    print("3. Press 'r' to reset calibration.")
    print("4. Press 'q' to quit.\n")

    state = "SEEKING_GEM"
    gripper_triggered_time = 0  # Non-blocking gripper timing

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        # ---------------------------------------------
        # STATE 1: HOMOGRAPHY CALIBRATION
        # ---------------------------------------------
        if homography_matrix is None:
            display_frame = frame.copy()
            labels = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
            for i, pt in enumerate(calibration_points):
                cv2.line(display_frame, (pt[0]-15, pt[1]), (pt[0]+15, pt[1]), (0, 255, 0), 2)
                cv2.line(display_frame, (pt[0], pt[1]-15), (pt[0], pt[1]+15), (0, 255, 0), 2)
                cv2.putText(display_frame, f"{i+1}. {labels[i]}", (pt[0]+10, pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            next_label = labels[len(calibration_points)] if len(calibration_points) < 4 else 'Done'
            cv2.putText(display_frame, f"Click 4 inner corners ({len(calibration_points)}/4) -> {next_label}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Arena", display_frame)

        # ---------------------------------------------
        # STATE 2: ACTIVE NAVIGATION
        # ---------------------------------------------
        else:
            warped = cv2.warpPerspective(frame, homography_matrix, (WARP_W, WARP_H))

            # Feed a DOWNSCALED copy to Roboflow thread (smaller = faster upload)
            with _rf_frame_lock:
                _latest_frame_for_rf = cv2.resize(warped, (RF_QUERY_W, RF_QUERY_H), interpolation=cv2.INTER_AREA)

            # 1. ArUco Robot Tracking (runs on full-res warp — fast, local)
            corners, ids, _ = detector.detectMarkers(warped)
            robot_pos = None
            robot_heading = 0.0

            if ids is not None and 30 in ids:
                idx = np.where(ids == 30)[0][0]
                marker_corners = corners[idx][0]

                cx = int(np.mean(marker_corners[:, 0]))
                cy = int(np.mean(marker_corners[:, 1]))
                robot_pos = (cx, cy)

                front_x = (marker_corners[0][0] + marker_corners[1][0]) / 2.0
                front_y = (marker_corners[0][1] + marker_corners[1][1]) / 2.0

                cv2.circle(warped, (cx, cy), 5, (255, 0, 0), -1)
                cv2.line(warped, (cx, cy), (int(front_x), int(front_y)), (0, 255, 0), 3)

                robot_heading = math.atan2(front_y - cy, front_x - cx)

            # 2. Draw Roboflow AI Target Outlines
            if roboflow_predictions:
                for pred in roboflow_predictions:
                    px = int(pred['x'])
                    py = int(pred['y'])
                    pw = int(pred['width'])
                    ph = int(pred['height'])
                    pclass = pred.get('class', pred.get('class_name', 'Object'))

                    color = get_color_for_class(pclass)
                    top_left = (px - pw//2, py - ph//2)
                    bottom_right = (px + pw//2, py + ph//2)

                    is_target = (roboflow_target is not None and abs(px - roboflow_target[0]) < 5 and abs(py - roboflow_target[1]) < 5)
                    if is_target:
                        color = (0, 0, 255)
                        thickness = 3
                    else:
                        thickness = 1

                    cv2.rectangle(warped, top_left, bottom_right, color, thickness)

                    (text_w, text_h), _ = cv2.getTextSize(pclass, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(warped, (top_left[0], top_left[1] - text_h - 6), (top_left[0] + text_w, top_left[1]), color, -1)
                    cv2.putText(warped, pclass, (top_left[0], top_left[1] - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if roboflow_target is not None:
                if robot_pos:
                    cv2.line(warped, robot_pos, roboflow_target, (0, 0, 255), 2, cv2.LINE_AA)

                if state == "SEEKING_GEM":
                    cv2.putText(warped, "AIMING (NEON RED)", (roboflow_target[0]-20, roboflow_target[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                elif state == "GRABBED":
                    cv2.putText(warped, "TARGET ACQUIRED", (roboflow_target[0]-20, roboflow_target[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 3. Closed-Loop Steering Math
            action_text = "WAITING FOR TARGET..."

            if robot_pos and roboflow_target and state == "SEEKING_GEM":
                dx = roboflow_target[0] - robot_pos[0]
                dy = roboflow_target[1] - robot_pos[1]
                distance_px = math.hypot(dx, dy)
                distance_mm = distance_px * MM_PER_PIXEL

                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - robot_heading
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

                cv2.putText(warped, f"Distance: {int(distance_mm)}mm | Heading Error: {angle_diff:.2f} rad", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # TERMINAL ACQUISITION (non-blocking)
                if distance_mm < PROXIMITY_THRESHOLD_MM:
                    action_text = "NEXT: TRIGGER GRIPPER (REACHED TARGET)"
                    print(">>> PROXIMITY TRIGGERED! GRABBING PAYLOAD! <<<")
                    send_udp('S')
                    send_udp('C')
                    gripper_triggered_time = time.time()
                    state = "GRABBED"

                # GLOBAL STEERING
                else:
                    if angle_diff > 0.3:
                        action_text = "NEXT: PIVOT RIGHT (ALIGNING)"
                        send_udp('R')
                    elif angle_diff < -0.3:
                        action_text = "NEXT: PIVOT LEFT (ALIGNING)"
                        send_udp('L')
                    else:
                        action_text = "NEXT: DRIVE FORWARD (APPROACHING)"
                        send_udp('F')
            else:
                send_udp('S')
                if state == "GRABBED":
                    action_text = "NEXT: PAYLOAD SECURED (WAITING COMMAND)"
                elif not robot_pos:
                    action_text = "WARNING: ROBOT ARUCO MARKER LOST!"

            # Draw HUD block at the bottom
            cv2.rectangle(warped, (0, WARP_H - 50), (WARP_W, WARP_H), (0, 0, 0), -1)
            cv2.putText(warped, action_text, (20, WARP_H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("Arena", warped)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            print(">>> Resetting Calibration <<<")
            homography_matrix = None
            calibration_points = []

    # Cleanup
    roboflow_active = False
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
