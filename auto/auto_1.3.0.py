# auto_1.3.0.py — Autonomous Pick & Place Brain
# Overhead webcam + ArUco ID 34 tracks rover.
# Roboflow detects gems & drop zones (background thread, non-blocking).
# Single-char UDP dispatch: F=Forward  L=Left  R=Right  S=Stop  C=Grab  D=Drop & Retreat
# ESP32 replies: GRABBED | DROPPED

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
ESP32_IP              = "10.218.230.31"
UDP_PORT              = 4210
CAMERA_INDEX          = 0
ROBOT_ARUCO_ID        = 34

WARP_W                = 800
WARP_H                = 600
MM_PER_PIXEL          = 2.5
PROXIMITY_THRESHOLD_MM = 150    # 15 cm → trigger grab / drop

RF_API_KEY   = "X5wKaATp2EknpnzoIAqF"
RF_WORKSPACE = "prem-supthaksina"
RF_WORKFLOW  = "arena-gem-and-drop-zone-detectio"
RF_QUERY_W   = 640
RF_QUERY_H   = 480

ALIGN_THRESHOLD = 0.26          # ~15 degrees — below this drives forward

# ==========================================
#               GLOBALS
# ==========================================
calibration_points = []
homography_matrix  = None

_rf_frame           = None
_rf_lock            = threading.Lock()
roboflow_active     = True
roboflow_predictions = []
roboflow_gem_targets  = []    # list of (cx, cy, cls)
roboflow_drop_targets = []    # list of (cx, cy, cls)

HAS_GEM = False

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
_last_cmd      = None
_last_cmd_time = 0.0

# ==========================================
#               CAMERA
# ==========================================
class FastCapture:
    """Threaded camera reader — always returns the latest frame with no buffer lag."""
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self._lock    = threading.Lock()
        self._stopped = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stopped:
            ret, frame = self.cap.read()
            with self._lock:
                self.ret, self.frame = ret, frame

    def read(self):
        with self._lock:
            return self.ret, (self.frame.copy() if self.frame is not None else None)

    def release(self):
        self._stopped = True
        self.cap.release()

# ==========================================
#               HELPERS
# ==========================================
def send_udp(cmd: str):
    """Send single-char command at max 33 Hz, skip duplicates."""
    global _last_cmd, _last_cmd_time
    now = time.time()
    if cmd != _last_cmd or (now - _last_cmd_time) > 0.03:
        try:
            sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
        except Exception:
            pass
        _last_cmd      = cmd
        _last_cmd_time = now

def is_drop_zone(cls: str) -> bool:
    n = cls.lower()
    return "drop" in n or "zone" in n or "area" in n or "base" in n

def extract_predictions(data) -> list:
    """Recursively pull {x, y, width, height, class} dicts out of any Roboflow response shape."""
    preds = []
    if hasattr(data, "predictions"):
        for p in data.predictions:
            preds.append({
                "x":      getattr(p, "x", 0),
                "y":      getattr(p, "y", 0),
                "width":  getattr(p, "width", 0),
                "height": getattr(p, "height", 0),
                "class":  getattr(p, "class_name", getattr(p, "class", "Object")),
            })
    elif isinstance(data, dict):
        if "x" in data and "y" in data:
            preds.append(data)
        elif "center_x" in data:
            d = dict(data); d["x"] = data["center_x"]; d["y"] = data["center_y"]
            preds.append(d)
        for v in data.values():
            preds.extend(extract_predictions(v))
    elif isinstance(data, list):
        for item in data:
            preds.extend(extract_predictions(item))
    return preds

def scale_preds(preds: list, sx: float, sy: float) -> list:
    """Scale prediction coordinates from RF resolution to warp resolution."""
    out = []
    for p in preds:
        sp = dict(p)
        sp["x"] = p["x"] * sx; sp["y"] = p["y"] * sy
        sp["width"] = p["width"] * sx; sp["height"] = p["height"] * sy
        out.append(sp)
    return out

def order_points(pts):
    xs = pts[np.argsort(pts[:, 0]), :]
    left  = xs[:2, :][np.argsort(xs[:2, 1]), :]
    right = xs[2:, :][np.argsort(xs[2:, 1]), :]
    return np.array([left[0], right[0], right[1], left[1]], dtype="float32")

def refine_centroid_hsv(img, px, py, pw, ph):
    """HSV-mask the bounding box to find the exact coloured centroid, not the bbox centre."""
    x1 = max(0, int(px - pw // 2)); y1 = max(0, int(py - ph // 2))
    x2 = min(WARP_W, int(px + pw // 2)); y2 = min(WARP_H, int(py + ph // 2))
    roi = img[y1:y2, x1:x2]
    if roi.size == 0:
        return int(px), int(py)
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 45, 45]), np.array([180, 255, 255]))
    M    = cv2.moments(mask)
    if M["m00"] > 20:
        return int(M["m10"] / M["m00"]) + x1, int(M["m01"] / M["m00"]) + y1
    return int(px), int(py)

# ==========================================
#            INFERENCE THREAD
# ==========================================
def inference_worker():
    global roboflow_predictions, roboflow_gem_targets, roboflow_drop_targets

    client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",
        api_key=RF_API_KEY,
    ).configure(InferenceConfiguration(api_key_transport="header"))

    while roboflow_active:
        with _rf_lock:
            snap = _rf_frame.copy() if _rf_frame is not None else None

        if snap is None:
            time.sleep(0.02)
            continue

        try:
            small = cv2.resize(snap, (RF_QUERY_W, RF_QUERY_H), interpolation=cv2.INTER_AREA)
            res   = client.run_workflow(
                workspace_name=RF_WORKSPACE,
                workflow_id=RF_WORKFLOW,
                images={"image": small},
                use_cache=True,
            )
            sx, sy = WARP_W / RF_QUERY_W, WARP_H / RF_QUERY_H
            scaled = scale_preds(extract_predictions(res), sx, sy)

            gems, drops = [], []
            for p in scaled:
                cls      = p.get("class", p.get("class_name", "Object"))
                cx, cy   = refine_centroid_hsv(snap, p["x"], p["y"], p["width"], p["height"])
                p["cx"]  = cx; p["cy"] = cy
                (drops if is_drop_zone(cls) else gems).append((cx, cy, cls))

            roboflow_predictions  = scaled
            roboflow_gem_targets  = gems
            roboflow_drop_targets = drops

        except Exception as e:
            print(f"[AI] {e}")

        time.sleep(0.01)

# ==========================================
#            MOUSE CALLBACK
# ==========================================
def mouse_callback(event, x, y, flags, param):
    global calibration_points, homography_matrix
    if event == cv2.EVENT_LBUTTONDOWN and len(calibration_points) < 4:
        calibration_points.append((x, y))
        print(f"  Corner {len(calibration_points)}: ({x}, {y})")
        if len(calibration_points) == 4:
            src = order_points(np.array(calibration_points, dtype=np.float32))
            dst = np.array([[0,0],[WARP_W,0],[WARP_W,WARP_H],[0,WARP_H]], dtype=np.float32)
            homography_matrix = cv2.getPerspectiveTransform(src, dst)
            print(">>> Homography set — autonomous navigation active <<<")

# ==========================================
#                   MAIN
# ==========================================
def main():
    global _rf_frame, roboflow_active, homography_matrix, calibration_points, HAS_GEM

    cap = FastCapture(CAMERA_INDEX)
    cv2.namedWindow("Arena")
    cv2.setMouseCallback("Arena", mouse_callback)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector   = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())

    threading.Thread(target=inference_worker, daemon=True).start()

    state       = "SEEKING_GEM"
    HAS_GEM     = False
    state_timer = 0.0

    print("\n--- Click 4 arena corners to calibrate. Press 'r' to reset, 'q' to quit. ---\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        # ── CALIBRATION ────────────────────────────────────────────────────────
        if homography_matrix is None:
            disp = frame.copy()
            for i, pt in enumerate(calibration_points):
                cv2.drawMarker(disp, pt, (0,255,0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(disp, str(i+1), (pt[0]+10, pt[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(disp, f"Click 4 arena corners ({len(calibration_points)}/4)",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow("Arena", disp)

        # ── AUTONOMOUS NAVIGATION ──────────────────────────────────────────────
        else:
            warped = cv2.warpPerspective(frame, homography_matrix, (WARP_W, WARP_H))
            with _rf_lock:
                _rf_frame = warped

            # A. Receive ESP32 UDP confirmations (GRABBED / DROPPED)
            try:
                while True:
                    msg = sock.recvfrom(256)[0].decode(errors="ignore").strip()
                    if "GRABBED" in msg and state == "GRABBING_GEM":
                        HAS_GEM, state = True, "SEEKING_DROP"
                        print(">>> [UDP] GRABBED — seeking drop zone <<<")
                    elif "DROPPED" in msg and state == "DROPPING_PAYLOAD":
                        HAS_GEM, state = False, "SEEKING_GEM"
                        print(">>> [UDP] DROPPED — hunting next gem <<<")
            except BlockingIOError:
                pass

            # Safety timeouts if UDP reply is lost
            if state == "GRABBING_GEM"    and (time.time() - state_timer) > 0.65:
                HAS_GEM, state = True, "SEEKING_DROP"
                print(">>> [Timer] Grab timeout — seeking drop zone <<<")
            elif state == "DROPPING_PAYLOAD" and (time.time() - state_timer) > 1.35:
                HAS_GEM, state = False, "SEEKING_GEM"
                print(">>> [Timer] Drop timeout — hunting next gem <<<")

            # B. ArUco tracking (Robot ID 34, ~60 FPS)
            corners, ids, _ = detector.detectMarkers(warped)
            robot_pos    = None
            robot_heading = 0.0

            if ids is not None and ROBOT_ARUCO_ID in ids:
                idx = np.where(ids == ROBOT_ARUCO_ID)[0][0]
                mc  = corners[idx][0]
                cx, cy = int(np.mean(mc[:,0])), int(np.mean(mc[:,1]))
                robot_pos = (cx, cy)
                fx = (mc[0][0] + mc[1][0]) / 2.0
                fy = (mc[0][1] + mc[1][1]) / 2.0
                robot_heading = math.atan2(fy - cy, fx - cx)
                cv2.circle(warped, (cx, cy), 6, (255,0,0), -1)
                cv2.line(warped, (cx, cy), (int(fx), int(fy)), (0,255,0), 3)

            # C. Draw detections
            for pred in roboflow_predictions:
                px   = int(pred.get("cx", pred["x"]))
                py   = int(pred.get("cy", pred["y"]))
                pw   = int(pred["width"]); ph = int(pred["height"])
                cls  = pred.get("class", pred.get("class_name", "Object"))
                tl   = (int(pred["x"] - pw//2), int(pred["y"] - ph//2))
                br   = (int(pred["x"] + pw//2), int(pred["y"] + ph//2))
                if is_drop_zone(cls):
                    cv2.rectangle(warped, tl, br, (255,255,0), 2)
                    cv2.putText(warped, f"[DROP] {cls}", (tl[0], tl[1]-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,0), 1)
                else:
                    cv2.rectangle(warped, tl, br, (0,165,255), 2)
                    cv2.circle(warped, (px,py), 4, (0,0,255), -1)
                    cv2.putText(warped, f"[GEM] {cls}", (tl[0], tl[1]-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,165,255), 1)

            # D. Select nearest target
            active_target = None
            if robot_pos:
                pool = roboflow_drop_targets if HAS_GEM else roboflow_gem_targets
                if pool:
                    nearest = min(pool, key=lambda t: math.hypot(t[0]-robot_pos[0], t[1]-robot_pos[1]))
                    active_target = (nearest[0], nearest[1])

            # E. Trajectory line + UDP dispatch
            action = f"STATE: {state} | HAS_GEM: {HAS_GEM}"

            if robot_pos and active_target and state in ("SEEKING_GEM", "SEEKING_DROP"):
                color = (255,255,0) if HAS_GEM else (0,0,255)
                cv2.line(warped, robot_pos, active_target, color, 4 if HAS_GEM else 2, cv2.LINE_AA)

                dx     = active_target[0] - robot_pos[0]
                dy     = active_target[1] - robot_pos[1]
                dist   = math.hypot(dx, dy) * MM_PER_PIXEL
                err    = (math.atan2(dy, dx) - robot_heading + math.pi) % (2*math.pi) - math.pi

                cv2.putText(warped, f"Dist:{int(dist)}mm  Err:{math.degrees(err):.1f}deg",
                            (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)

                if dist < PROXIMITY_THRESHOLD_MM:
                    if state == "SEEKING_GEM":
                        send_udp("C"); state = "GRABBING_GEM"
                        state_timer = time.time(); action = "GRABBING GEM"
                    else:
                        send_udp("D"); state = "DROPPING_PAYLOAD"
                        state_timer = time.time(); action = "DROP & RETREAT"
                else:
                    if abs(err) > ALIGN_THRESHOLD:
                        cmd = "R" if err > 0 else "L"
                        send_udp(cmd); action = f"PIVOT {'RIGHT' if err>0 else 'LEFT'}"
                    else:
                        send_udp("F"); action = "DRIVE FORWARD"

            elif state in ("GRABBING_GEM", "DROPPING_PAYLOAD"):
                action = f"WAITING FOR ESP32 → {state}"

            else:
                send_udp("S")
                action = "ARUCO LOST" if not robot_pos else f"NO TARGET ({state})"

            # HUD bar
            cv2.rectangle(warped, (0, WARP_H-45), (WARP_W, WARP_H), (0,0,0), -1)
            cv2.putText(warped, action, (10, WARP_H-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)
            cv2.imshow("Arena", warped)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            send_udp("S"); break
        elif key == ord("r"):
            calibration_points[:] = []
            homography_matrix = None
            state, HAS_GEM = "SEEKING_GEM", False
            send_udp("O"); send_udp("S")
            print(">>> Reset <<<")

    roboflow_active = False
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
