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
ESP32_IP = "10.218.230.31"
UDP_PORT = 4210
CAMERA_INDEX = 0

WARP_W = 800          # Smaller warp = faster ArUco + less CPU
WARP_H = 600
MM_PER_PIXEL = 2.5
PROXIMITY_THRESHOLD_MM = 150

# Roboflow
RF_API_KEY = "X5wKaATp2EknpnzoIAqF"
RF_WORKSPACE = "prem-supthaksina"
RF_WORKFLOW = "arena-gem-and-drop-zone-detectio"
RF_QUERY_W = 640
RF_QUERY_H = 480

# Steering tuning
BASE_SPEED = 160        # Forward cruise speed
TURN_GAIN = 130         # How aggressively to correct heading
ALIGN_THRESHOLD = 0.25  # radians — below this, drive forward

# ==========================================
#               GLOBALS
# ==========================================
calibration_points = []
homography_matrix = None

# Roboflow shared state (written by RF thread, read by main)
_rf_frame = None
_rf_lock = threading.Lock()
roboflow_target = None       # Current navigation target (gem or drop zone)
roboflow_gem_target = None   # Closest gem position
roboflow_drop_target = None  # Closest drop zone position
roboflow_predictions = []
roboflow_active = True

# UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

# Keep track of last sent command to avoid spamming identical packets
_last_cmd_sent = None
_last_cmd_time = 0

# ==========================================
#          THREADED CAMERA CAPTURE
# ==========================================
class FastCapture:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Lower camera res = faster
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
            return self.ret, self.frame.copy() if self.frame is not None else (self.ret, None)

    def release(self):
        self._stopped = True
        self.cap.release()

# ==========================================
#               HELPERS
# ==========================================
def send_udp(cmd):
    """Send a UDP command, skipping duplicates within 30ms."""
    global _last_cmd_sent, _last_cmd_time
    now = time.time()
    # Always send if command changed, or if >30ms since last send
    if cmd != _last_cmd_sent or (now - _last_cmd_time) > 0.03:
        try:
            sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
        except Exception:
            pass
        _last_cmd_sent = cmd
        _last_cmd_time = now

def extract_roboflow_predictions(data):
    preds = []
    if isinstance(data, dict):
        if 'x' in data and 'y' in data and 'width' in data and 'height' in data:
            preds.append(data)
        elif 'center_x' in data and 'center_y' in data and 'width' in data and 'height' in data:
            dc = dict(data)
            dc['x'] = data['center_x']
            dc['y'] = data['center_y']
            preds.append(dc)
        for v in data.values():
            preds.extend(extract_roboflow_predictions(v))
    elif isinstance(data, list):
        for item in data:
            preds.extend(extract_roboflow_predictions(item))
    return preds

def get_color_for_class(name):
    n = name.lower()
    if 'red' in n: return (50, 50, 220)
    if 'blue' in n: return (255, 150, 50)
    if 'green' in n: return (50, 220, 50)
    if 'purple' in n: return (180, 50, 130)
    h = hash(name)
    return ((h & 0xFF), ((h >> 8) & 0xFF), ((h >> 16) & 0xFF))

def scale_preds(preds, sx, sy):
    out = []
    for p in preds:
        sp = dict(p)
        sp['x'] = p['x'] * sx
        sp['y'] = p['y'] * sy
        sp['width'] = p['width'] * sx
        sp['height'] = p['height'] * sy
        out.append(sp)
    return out

def order_points(pts):
    xs = pts[np.argsort(pts[:, 0]), :]
    left = xs[:2, :][np.argsort(xs[:2, 1]), :]
    right = xs[2:, :][np.argsort(xs[2:, 1]), :]
    return np.array([left[0], right[0], right[1], left[1]], dtype="float32")

def clamp_speed(s):
    """Ensure PWM is high enough to move the motors, capped at 200."""
    if s > 0: return max(110, min(200, s))
    if s < 0: return min(-110, max(-200, s))
    return 0

def is_drop_zone(class_name):
    """Returns True if this detection is a drop zone (not a gem)."""
    n = class_name.lower()
    return 'drop' in n or 'zone' in n or 'area' in n or 'base' in n

# ==========================================
#           ROBOFLOW BACKGROUND THREAD
# ==========================================
def roboflow_thread():
    global roboflow_target, roboflow_gem_target, roboflow_drop_target, roboflow_predictions
    client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",
        api_key=RF_API_KEY
    ).configure(InferenceConfiguration(api_key_transport="header"))

    print("[RF] Roboflow thread started")
    while roboflow_active:
        with _rf_lock:
            snap = _rf_frame

        if snap is None:
            time.sleep(0.05)
            continue

        try:
            result = client.run_workflow(
                workspace_name=RF_WORKSPACE,
                workflow_id=RF_WORKFLOW,
                images={"image": snap},
                use_cache=True
            )
            preds = extract_roboflow_predictions(result)
            sx, sy = WARP_W / RF_QUERY_W, WARP_H / RF_QUERY_H
            preds = scale_preds(preds, sx, sy)
            roboflow_predictions = preds

            # Separate gems from drop zones
            gems = []
            drops = []
            for p in preds:
                cls = p.get('class', p.get('class_name', 'Object'))
                if is_drop_zone(cls):
                    drops.append(p)
                else:
                    gems.append(p)

            roboflow_gem_target = (int(gems[0]['x']), int(gems[0]['y'])) if gems else None
            roboflow_drop_target = (int(drops[0]['x']), int(drops[0]['y'])) if drops else None

            # Log what we found
            print(f"[RF] Found {len(gems)} gems, {len(drops)} drop zones")

        except Exception as e:
            print(f"[RF] Error: {e}")

        time.sleep(0.01)

# ==========================================
#           MOUSE CALLBACK
# ==========================================
def mouse_callback(event, x, y, flags, param):
    global calibration_points, homography_matrix
    if event == cv2.EVENT_LBUTTONDOWN and len(calibration_points) < 4:
        calibration_points.append((x, y))
        print(f"Point {len(calibration_points)}: ({x}, {y})")
        if len(calibration_points) == 4:
            src = order_points(np.array(calibration_points, dtype=np.float32))
            dst = np.array([[0,0],[WARP_W,0],[WARP_W,WARP_H],[0,WARP_H]], dtype=np.float32)
            homography_matrix = cv2.getPerspectiveTransform(src, dst)
            print(">>> Homography set! AI Navigation active <<<")

# ==========================================
#               MAIN
# ==========================================
def main():
    global _rf_frame, roboflow_active, homography_matrix, calibration_points

    cap = FastCapture(CAMERA_INDEX)
    cv2.namedWindow("Arena")
    cv2.setMouseCallback("Arena", mouse_callback)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())

    threading.Thread(target=roboflow_thread, daemon=True).start()

    print("\n--- Click 4 corners, then AI drives. Press 'r' to reset, 'q' to quit. ---\n")

    state = "SEEKING_GEM"
    gripper_time = 0
    last_steer_cmd = 'S'  # Remember last steering command for when RF is stale

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        # ---- CALIBRATION PHASE ----
        if homography_matrix is None:
            disp = frame.copy()
            for i, pt in enumerate(calibration_points):
                cv2.drawMarker(disp, pt, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(disp, str(i+1), (pt[0]+10, pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(disp, f"Click corners ({len(calibration_points)}/4)", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow("Arena", disp)

        # ---- NAVIGATION PHASE ----
        else:
            warped = cv2.warpPerspective(frame, homography_matrix, (WARP_W, WARP_H))

            # Feed downscaled frame to Roboflow (non-blocking)
            with _rf_lock:
                _rf_frame = cv2.resize(warped, (RF_QUERY_W, RF_QUERY_H), interpolation=cv2.INTER_AREA)

            # --- ArUco tracking (fast, local) ---
            corners, ids, _ = detector.detectMarkers(warped)
            robot_pos = None
            robot_heading = 0.0

            if ids is not None and 30 in ids:
                idx = np.where(ids == 30)[0][0]
                mc = corners[idx][0]
                cx, cy = int(np.mean(mc[:, 0])), int(np.mean(mc[:, 1]))
                robot_pos = (cx, cy)
                fx = (mc[0][0] + mc[1][0]) / 2.0
                fy = (mc[0][1] + mc[1][1]) / 2.0
                cv2.circle(warped, (cx, cy), 5, (255, 0, 0), -1)
                cv2.line(warped, (cx, cy), (int(fx), int(fy)), (0, 255, 0), 3)
                robot_heading = math.atan2(fy - cy, fx - cx)

            # --- Draw Roboflow detections ---
            for pred in roboflow_predictions:
                px, py = int(pred['x']), int(pred['y'])
                pw, ph = int(pred['width']), int(pred['height'])
                cls = pred.get('class', pred.get('class_name', 'Object'))
                color = get_color_for_class(cls)
                tl = (px - pw//2, py - ph//2)
                br = (px + pw//2, py + ph//2)

                # Mark drop zones differently
                if is_drop_zone(cls):
                    cv2.rectangle(warped, tl, br, (255, 255, 0), 2)  # Cyan for drop zones
                    label = f"[DROP] {cls}"
                else:
                    cv2.rectangle(warped, tl, br, color, 2)
                    label = f"[GEM] {cls}"

                cv2.putText(warped, label, (tl[0], tl[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)

            # --- STEERING LOGIC ---
            action_text = "WAITING FOR TARGET..."

            # Select target based on current state
            if state == "SEEKING_GEM":
                roboflow_target = roboflow_gem_target
            elif state == "SEEKING_DROP":
                roboflow_target = roboflow_drop_target
            else:
                roboflow_target = None

            # Draw aiming line
            if roboflow_target and robot_pos:
                cv2.line(warped, robot_pos, roboflow_target, (0,0,255), 2, cv2.LINE_AA)

            if robot_pos and roboflow_target and state in ("SEEKING_GEM", "SEEKING_DROP"):
                dx = roboflow_target[0] - robot_pos[0]
                dy = roboflow_target[1] - robot_pos[1]
                dist_px = math.hypot(dx, dy)
                dist_mm = dist_px * MM_PER_PIXEL

                target_angle = math.atan2(dy, dx)
                err = target_angle - robot_heading
                err = (err + math.pi) % (2 * math.pi) - math.pi

                state_label = "GEM" if state == "SEEKING_GEM" else "DROP"
                cv2.putText(warped, f"[{state_label}] Dist: {int(dist_mm)}mm  Err: {err:.2f}rad", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

                if dist_mm < PROXIMITY_THRESHOLD_MM:
                    if state == "SEEKING_GEM":
                        # GRAB the gem
                        action_text = "GRABBING GEM!"
                        send_udp('S')
                        time.sleep(0.05)
                        send_udp('C')
                        gripper_time = time.time()
                        state = "SEEKING_DROP"
                        last_steer_cmd = 'S'
                        print(">>> GEM GRABBED! Now seeking drop zone <<<")
                    elif state == "SEEKING_DROP":
                        # DROP at the zone
                        action_text = "DROPPING PAYLOAD!"
                        send_udp('S')
                        time.sleep(0.05)
                        send_udp('O')
                        state = "DELIVERED"
                        last_steer_cmd = 'S'
                        print(">>> DELIVERED! Mission complete <<<")
                else:
                    # Proportional steering toward current target
                    turn = int(err * TURN_GAIN)
                    ls = BASE_SPEED + turn
                    rs = BASE_SPEED - turn
                    ls = clamp_speed(ls)
                    rs = clamp_speed(rs)

                    cmd_str = f"M,{ls},{rs}"
                    send_udp(cmd_str)
                    last_steer_cmd = cmd_str

                    if abs(err) > ALIGN_THRESHOLD:
                        action_text = f"[{state_label}] PIVOT L:{ls} R:{rs}"
                    else:
                        action_text = f"[{state_label}] DRIVE L:{ls} R:{rs}"

            elif robot_pos and not roboflow_target and state in ("SEEKING_GEM", "SEEKING_DROP"):
                if last_steer_cmd != 'S':
                    send_udp(last_steer_cmd)
                    action_text = f"COASTING (RF pending)"
                else:
                    send_udp('S')
                    tgt_type = "gem" if state == "SEEKING_GEM" else "drop zone"
                    action_text = f"NO {tgt_type.upper()} FOUND — STOPPED"

            elif state == "DELIVERED":
                send_udp('S')
                action_text = "MISSION COMPLETE! Press 'r' to restart"

            elif not robot_pos:
                send_udp('S')
                action_text = "WARNING: ARUCO LOST!"
                last_steer_cmd = 'S'

            # HUD bar
            cv2.rectangle(warped, (0, WARP_H-45), (WARP_W, WARP_H), (0,0,0), -1)
            cv2.putText(warped, action_text, (15, WARP_H-12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

            cv2.imshow("Arena", warped)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            print(">>> Reset <<<")
            homography_matrix = None
            calibration_points = []
            state = "SEEKING_GEM"
            last_steer_cmd = 'S'

    roboflow_active = False
    send_udp('S')
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
