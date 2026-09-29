import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import time
import math
import threading

# ==========================================
#      LOCAL INFERENCE vs SDK FALLBACK
# ==========================================
try:
    from inference import get_model
    USE_LOCAL_INFERENCE = True
    print("[INIT] Using local RAM inference (`from inference import get_model`)")
except ImportError:
    from inference_sdk import InferenceHTTPClient, InferenceConfiguration
    USE_LOCAL_INFERENCE = False
    print("[INIT] `inference` local runtime not found on Python 3.14; using non-blocking cached background inference.")

# ==========================================
#               CONFIGURATION
# ==========================================
ESP32_IP = "10.218.230.31"
UDP_PORT = 4210
CAMERA_INDEX = 0
ROBOT_ARUCO_ID = 34

WARP_W = 800
WARP_H = 600
MM_PER_PIXEL = 2.5
PROXIMITY_THRESHOLD_MM = 150  # 15 cm trigger distance

# Roboflow Model & Credentials
RF_API_KEY = "X5wKaATp2EknpnzoIAqF"
RF_WORKSPACE = "prem-supthaksina"
RF_WORKFLOW = "arena-gem-and-drop-zone-detectio"
RF_MODEL_ID = "arena-gem-and-drop-zone-detectio/1"
RF_QUERY_W = 640
RF_QUERY_H = 480

# Steering Alignment Threshold (~15 degrees = 0.26 rad)
ALIGN_THRESHOLD = 0.26

# ==========================================
#               GLOBALS
# ==========================================
calibration_points = []
homography_matrix = None

_rf_frame = None
_rf_lock = threading.Lock()
roboflow_active = True
roboflow_predictions = []
roboflow_gem_targets = []
roboflow_drop_targets = []

# State Machine Globals
HAS_GEM = False

# Non-blocking UDP Socket (sends single-char commands & receives GRABBED / DROPPED)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
_last_cmd_sent = None
_last_cmd_time = 0.0

# ==========================================
#        ZERO-LAG CAMERA CAPTURE
# ==========================================
class FastCapture:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self._lock = threading.Lock()
        self._stopped = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stopped:
            ret, frame = self.cap.read()
            with self._lock:
                self.ret, self.frame = ret, frame

    def read(self):
        with self._lock:
            return self.ret, self.frame.copy() if self.frame is not None else (self.ret, None)

    def release(self):
        self._stopped = True
        self.cap.release()

# ==========================================
#               HELPERS
# ==========================================
def send_udp(cmd: str):
    """Send a single-character UDP command ('F','B','L','R','S','C','D') at ~33Hz max."""
    global _last_cmd_sent, _last_cmd_time
    now = time.time()
    if cmd != _last_cmd_sent or (now - _last_cmd_time) > 0.03:
        try:
            sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
        except Exception:
            pass
        _last_cmd_sent = cmd
        _last_cmd_time = now

def is_drop_zone(class_name: str) -> bool:
    n = class_name.lower()
    return "drop" in n or "zone" in n or "area" in n or "base" in n

def extract_predictions(data):
    preds = []
    if hasattr(data, "predictions"):
        for p in data.predictions:
            preds.append({
                "x": getattr(p, "x", 0),
                "y": getattr(p, "y", 0),
                "width": getattr(p, "width", 0),
                "height": getattr(p, "height", 0),
                "class": getattr(p, "class_name", getattr(p, "class", "Object")),
            })
    elif isinstance(data, dict):
        if "x" in data and "y" in data and "width" in data and "height" in data:
            preds.append(data)
        elif "center_x" in data and "center_y" in data and "width" in data and "height" in data:
            dc = dict(data)
            dc["x"] = data["center_x"]
            dc["y"] = data["center_y"]
            preds.append(dc)
        for v in data.values():
            preds.extend(extract_predictions(v))
    elif isinstance(data, list):
        for item in data:
            preds.extend(extract_predictions(item))
    return preds

def scale_preds(preds, sx, sy):
    out = []
    for p in preds:
        sp = dict(p)
        sp["x"] = p["x"] * sx
        sp["y"] = p["y"] * sy
        sp["width"] = p["width"] * sx
        sp["height"] = p["height"] * sy
        out.append(sp)
    return out

def order_points(pts):
    xs = pts[np.argsort(pts[:, 0]), :]
    left = xs[:2, :][np.argsort(xs[:2, 1]), :]
    right = xs[2:, :][np.argsort(xs[2:, 1]), :]
    return np.array([left[0], right[0], right[1], left[1]], dtype="float32")

def refine_centroid_hsv(warped_img, px, py, pw, ph):
    """Apply HSV saturation/value masking inside bounding box for exact physical centroid."""
    x1 = max(0, int(px - pw // 2))
    y1 = max(0, int(py - ph // 2))
    x2 = min(WARP_W, int(px + pw // 2))
    y2 = min(WARP_H, int(py + ph // 2))
    roi = warped_img[y1:y2, x1:x2]
    if roi.size == 0:
        return int(px), int(py)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Mask colored objects against the neutral wooden arena floor (S > 45, V > 45)
    mask = cv2.inRange(hsv, np.array([0, 45, 45]), np.array([180, 255, 255]))
    M = cv2.moments(mask)
    if M["m00"] > 20:
        cx = int(M["m10"] / M["m00"]) + x1
        cy = int(M["m01"] / M["m00"]) + y1
        return cx, cy
    return int(px), int(py)

# ==========================================
#          INFERENCE WORKER THREAD
# ==========================================
def inference_worker():
    global roboflow_predictions, roboflow_gem_targets, roboflow_drop_targets

    local_model = None
    cloud_client = None

    if USE_LOCAL_INFERENCE:
        try:
            local_model = get_model(model_id=RF_MODEL_ID, api_key=RF_API_KEY)
            print("[AI] Local model loaded into RAM successfully!")
        except Exception as e:
            print(f"[AI] Local model load error ({e}), falling back to background client.")

    if local_model is None:
        from inference_sdk import InferenceHTTPClient, InferenceConfiguration
        cloud_client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key=RF_API_KEY
        ).configure(InferenceConfiguration(api_key_transport="header"))

    while roboflow_active:
        with _rf_lock:
            snap = _rf_frame.copy() if _rf_frame is not None else None

        if snap is None:
            time.sleep(0.02)
            continue

        try:
            small = cv2.resize(snap, (RF_QUERY_W, RF_QUERY_H), interpolation=cv2.INTER_AREA)
            if local_model is not None:
                res = local_model.infer(small)
                raw_preds = extract_predictions(res[0] if isinstance(res, list) else res)
            else:
                res = cloud_client.run_workflow(
                    workspace_name=RF_WORKSPACE,
                    workflow_id=RF_WORKFLOW,
                    images={"image": small},
                    use_cache=True
                )
                raw_preds = extract_predictions(res)

            sx, sy = WARP_W / RF_QUERY_W, WARP_H / RF_QUERY_H
            scaled = scale_preds(raw_preds, sx, sy)

            gems = []
            drops = []
            for p in scaled:
                cls = p.get("class", p.get("class_name", "Object"))
                cx, cy = refine_centroid_hsv(snap, p["x"], p["y"], p["width"], p["height"])
                p["cx"], p["cy"] = cx, cy
                if is_drop_zone(cls):
                    drops.append((cx, cy, cls))
                else:
                    gems.append((cx, cy, cls))

            roboflow_predictions = scaled
            roboflow_gem_targets = gems
            roboflow_drop_targets = drops

        except Exception as e:
            print(f"[AI] Inference warning: {e}")

        time.sleep(0.01)

# ==========================================
#           MOUSE CALLBACK
# ==========================================
def mouse_callback(event, x, y, flags, param):
    global calibration_points, homography_matrix
    if event == cv2.EVENT_LBUTTONDOWN and len(calibration_points) < 4:
        calibration_points.append((x, y))
        print(f"Calibration Point {len(calibration_points)}: ({x}, {y})")
        if len(calibration_points) == 4:
            src = order_points(np.array(calibration_points, dtype=np.float32))
            dst = np.array([[0, 0], [WARP_W, 0], [WARP_W, WARP_H], [0, WARP_H]], dtype=np.float32)
            homography_matrix = cv2.getPerspectiveTransform(src, dst)
            print(">>> Homography Calibrated! Autonomous Pick & Place Active <<<")

# ==========================================
#               MAIN LOOP
# ==========================================
def main():
    global _rf_frame, roboflow_active, homography_matrix, calibration_points, HAS_GEM

    cap = FastCapture(CAMERA_INDEX)
    cv2.namedWindow("Arena")
    cv2.setMouseCallback("Arena", mouse_callback)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())

    threading.Thread(target=inference_worker, daemon=True).start()

    state = "SEEKING_GEM"
    HAS_GEM = False
    state_timer = 0.0

    print("\n--- Click 4 Arena Corners to start. Press 'r' to reset state, 'q' to quit. ---\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        # ---- 1. CALIBRATION PHASE ----
        if homography_matrix is None:
            disp = frame.copy()
            for i, pt in enumerate(calibration_points):
                cv2.drawMarker(disp, pt, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(disp, str(i + 1), (pt[0] + 10, pt[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(disp, f"Click 4 Arena Corners ({len(calibration_points)}/4)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Arena", disp)

        # ---- 2. AUTONOMOUS NAVIGATION PHASE ----
        else:
            warped = cv2.warpPerspective(frame, homography_matrix, (WARP_W, WARP_H))

            with _rf_lock:
                _rf_frame = warped

            # --- A. Listen for ESP32 UDP Handshakes (GRABBED / DROPPED) ---
            try:
                while True:
                    data, _ = sock.recvfrom(1024)
                    msg = data.decode(errors="ignore").strip()
                    if "GRABBED" in msg and state == "GRABBING_GEM":
                        HAS_GEM = True
                        state = "SEEKING_DROP"
                        print(">>> [UDP] GRABBED received! HAS_GEM = True -> Navigating to Drop Zone <<<")
                    elif "DROPPED" in msg and state == "DROPPING_PAYLOAD":
                        HAS_GEM = False
                        state = "SEEKING_GEM"
                        print(">>> [UDP] DROPPED received! HAS_GEM = False -> Hunting next Gem <<<")
            except BlockingIOError:
                pass

            # Safety timeouts in case a Wi-Fi UDP reply packet drops
            if state == "GRABBING_GEM" and (time.time() - state_timer) > 0.65:
                HAS_GEM = True
                state = "SEEKING_DROP"
                print(">>> [Timer] Grab complete! HAS_GEM = True -> Navigating to Drop Zone <<<")
            elif state == "DROPPING_PAYLOAD" and (time.time() - state_timer) > 1.35:
                HAS_GEM = False
                state = "SEEKING_GEM"
                print(">>> [Timer] Drop & Retreat complete! HAS_GEM = False -> Hunting next Gem <<<")

            # --- B. Local 60 FPS ArUco Tracking (ID 34) ---
            corners, ids, _ = detector.detectMarkers(warped)
            robot_pos = None
            robot_heading = 0.0

            if ids is not None and ROBOT_ARUCO_ID in ids:
                idx = np.where(ids == ROBOT_ARUCO_ID)[0][0]
                mc = corners[idx][0]
                cx, cy = int(np.mean(mc[:, 0])), int(np.mean(mc[:, 1]))
                robot_pos = (cx, cy)
                fx = (mc[0][0] + mc[1][0]) / 2.0
                fy = (mc[0][1] + mc[1][1]) / 2.0
                robot_heading = math.atan2(fy - cy, fx - cx)

                cv2.circle(warped, (cx, cy), 6, (255, 0, 0), -1)
                cv2.line(warped, (cx, cy), (int(fx), int(fy)), (0, 255, 0), 3)

            # --- C. Draw Detections & Select Nearest Target ---
            for pred in roboflow_predictions:
                px, py = int(pred.get("cx", pred["x"])), int(pred.get("cy", pred["y"]))
                pw, ph = int(pred["width"]), int(pred["height"])
                cls = pred.get("class", pred.get("class_name", "Object"))
                tl = (int(pred["x"] - pw // 2), int(pred["y"] - ph // 2))
                br = (int(pred["x"] + pw // 2), int(pred["y"] + ph // 2))

                if is_drop_zone(cls):
                    cv2.rectangle(warped, tl, br, (255, 255, 0), 2)
                    cv2.putText(warped, f"[DROP] {cls}", (tl[0], tl[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                else:
                    cv2.rectangle(warped, tl, br, (0, 165, 255), 2)
                    cv2.circle(warped, (px, py), 4, (0, 0, 255), -1)
                    cv2.putText(warped, f"[GEM] {cls}", (tl[0], tl[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)

            active_target = None
            if robot_pos:
                if not HAS_GEM and roboflow_gem_targets:
                    # Pick closest gem to robot
                    nearest_gem = min(
                        roboflow_gem_targets,
                        key=lambda g: math.hypot(g[0] - robot_pos[0], g[1] - robot_pos[1])
                    )
                    active_target = (nearest_gem[0], nearest_gem[1])
                elif HAS_GEM and roboflow_drop_targets:
                    # Pick closest drop zone to robot
                    nearest_drop = min(
                        roboflow_drop_targets,
                        key=lambda d: math.hypot(d[0] - robot_pos[0], d[1] - robot_pos[1])
                    )
                    active_target = (nearest_drop[0], nearest_drop[1])

            # --- D. Trajectory Visualization & Single-Char UDP Dispatch ---
            action_text = f"STATE: {state} | HAS_GEM: {HAS_GEM}"

            if robot_pos and active_target and state in ("SEEKING_GEM", "SEEKING_DROP"):
                # Draw Trajectory Line (Cyan when HAS_GEM=True, Red when hunting Gem)
                line_color = (255, 255, 0) if HAS_GEM else (0, 0, 255)
                line_thickness = 4 if HAS_GEM else 2
                cv2.line(warped, robot_pos, active_target, line_color, line_thickness, cv2.LINE_AA)

                dx = active_target[0] - robot_pos[0]
                dy = active_target[1] - robot_pos[1]
                dist_mm = math.hypot(dx, dy) * MM_PER_PIXEL

                target_angle = math.atan2(dy, dx)
                err = (target_angle - robot_heading + math.pi) % (2 * math.pi) - math.pi

                cv2.putText(warped, f"Dist: {int(dist_mm)}mm | AngleErr: {math.degrees(err):.1f} deg",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

                if dist_mm < PROXIMITY_THRESHOLD_MM:
                    if state == "SEEKING_GEM":
                        send_udp("C")
                        state = "GRABBING_GEM"
                        state_timer = time.time()
                        action_text = "GRABBING GEM ('C')"
                    elif state == "SEEKING_DROP":
                        send_udp("D")
                        state = "DROPPING_PAYLOAD"
                        state_timer = time.time()
                        action_text = "RELEASE & RETREAT ('D')"
                else:
                    # Single-Char Fixed-Power Dispatch (Matches working Manual Mode protocol)
                    if abs(err) > ALIGN_THRESHOLD:
                        if err > 0:
                            send_udp("R")
                            action_text = f"[{state}] PIVOT RIGHT ('R')"
                        else:
                            send_udp("L")
                            action_text = f"[{state}] PIVOT LEFT ('L')"
                    else:
                        send_udp("F")
                        action_text = f"[{state}] DRIVE FORWARD ('F')"

            elif state in ("GRABBING_GEM", "DROPPING_PAYLOAD"):
                action_text = f"EXECUTING {state} ON ESP32..."

            else:
                send_udp("S")
                if not robot_pos:
                    action_text = "WARNING: ARUCO ID 34 LOST ('S')"
                else:
                    action_text = f"WAITING FOR TARGET ({state}) ('S')"

            # HUD Bar
            cv2.rectangle(warped, (0, WARP_H - 45), (WARP_W, WARP_H), (0, 0, 0), -1)
            cv2.putText(warped, action_text, (10, WARP_H - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.imshow("Arena", warped)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            send_udp("S")
            break
        elif key == ord("r"):
            calibration_points = []
            homography_matrix = None
            state = "SEEKING_GEM"
            HAS_GEM = False
            send_udp("O")
            send_udp("S")
            print(">>> Reset calibration and state machine <<<")

    roboflow_active = False
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
