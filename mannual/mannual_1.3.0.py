# mannual_1.3.0.py — MediaPipe Hand Gesture Controller
# Sends single-char UDP commands to ESP32 rover.
# F=Forward  B=Reverse  L=Left  R=Right  S=Stop  C=Close Servo  O=Open Servo

import cv2
import mediapipe as mp
import socket
import time

# ==========================================
#               CONFIGURATION
# ==========================================
CONFIG = {
    "ESP32_IP":         "10.218.230.31",
    "UDP_PORT":         4210,
    "CAMERA_INDEX":     0,
    "FPS_CAP_DELAY":    0.033,    # ~30 Hz UDP send rate
    "ARM_DEBOUNCE_SEC": 1.0,      # Seconds between servo commands
    "MP_CONFIDENCE":    0.8,
}

# ==========================================
#               VIDEO STREAM
# ==========================================
class VideoStream:
    """Threaded camera capture to prevent frame lag."""
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.last_ts_ms = 0
        import threading
        self._lock = threading.Lock()
        self._stopped = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stopped:
            ret, frame = self.cap.read()
            with self._lock:
                self.ret, self.frame = ret, frame

    def read(self):
        import threading
        with self._lock:
            return self.ret, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self._stopped = True
        self.cap.release()

# ==========================================
#            GESTURE DETECTION
# ==========================================
def detect_fingers(landmarks, hand_side):
    """Return dict of which fingers are up for the given hand."""
    TIP = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
    PIP = {"thumb": 3, "index": 7, "middle": 11, "ring": 15, "pinky": 19}

    fingers = {}
    for name, tip_id in TIP.items():
        tip = landmarks[tip_id]
        pip = landmarks[PIP[name]]
        if name == "thumb":
            # Thumb: compare x-axis (flipped for left/right hand)
            if hand_side == "Right":
                fingers["thumb"] = tip.x < pip.x
            else:
                fingers["thumb"] = tip.x > pip.x
        else:
            fingers[name] = tip.y < pip.y
    return fingers

# Gesture predicates
def is_L_shape(f):
    """Thumb + Index only."""
    return f["thumb"] and f["index"] and not f["middle"] and not f["ring"] and not f["pinky"]

def is_imp_combo(f):
    """Index + Middle + Pinky (no ring)."""
    return f["index"] and f["middle"] and f["pinky"] and not f["ring"]

def is_four_fingers(f):
    """Index + Middle + Ring + Pinky, thumb down."""
    return not f["thumb"] and f["index"] and f["middle"] and f["ring"] and f["pinky"]

def is_five(f):
    """All 5 fingers up."""
    return all(f.values())

# ==========================================
#            COMMAND MAPPING
# ==========================================
def map_gestures(left_fingers, right_fingers, last_arm_time):
    """Map hand gestures to a single UDP command character."""
    cmd, label = 'S', "STOP"
    new_arm_time = last_arm_time
    L, R = left_fingers is not None, right_fingers is not None

    if L and R:
        if is_five(left_fingers) and is_five(right_fingers):
            cmd, label = 'F', "FORWARD (5+5)"
        elif is_imp_combo(left_fingers) and is_imp_combo(right_fingers):
            cmd, label = 'B', "REVERSE (I+M+P)"
        elif is_four_fingers(left_fingers):
            cmd, label = 'L', "LEFT (4 fingers L)"
        elif is_four_fingers(right_fingers):
            cmd, label = 'R', "RIGHT (4 fingers R)"
        elif is_L_shape(left_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label = 'O', "OPEN SERVO (L-shape L)"
                new_arm_time = time.time()
        elif is_L_shape(right_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label = 'C', "CLOSE SERVO (L-shape R)"
                new_arm_time = time.time()

    elif L:
        if is_five(left_fingers):
            cmd, label = 'S', "STOP (5 L)"
        elif is_four_fingers(left_fingers):
            cmd, label = 'L', "LEFT (4 fingers L)"
        elif is_L_shape(left_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label = 'O', "OPEN SERVO (L-shape L)"
                new_arm_time = time.time()

    elif R:
        if is_five(right_fingers):
            cmd, label = 'S', "STOP (5 R)"
        elif is_four_fingers(right_fingers):
            cmd, label = 'R', "RIGHT (4 fingers R)"
        elif is_L_shape(right_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label = 'C', "CLOSE SERVO (L-shape R)"
                new_arm_time = time.time()

    return cmd, label, new_arm_time

# ==========================================
#                   MAIN
# ==========================================
def main():
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setblocking(False)

    mp_vision = mp.tasks.vision
    options = mp_vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path='hand_landmarker.task'),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=CONFIG["MP_CONFIDENCE"],
        min_hand_presence_confidence=CONFIG["MP_CONFIDENCE"],
        min_tracking_confidence=CONFIG["MP_CONFIDENCE"],
    )
    hands = mp_vision.HandLandmarker.create_from_options(options)

    print("Starting camera...")
    vs = VideoStream(src=CONFIG["CAMERA_INDEX"])
    time.sleep(1.0)

    arm_debounce_time = 0.0
    last_udp_time = 0.0
    start_time = time.time()

    print("Ready — press 'q' to quit.")
    print("Gestures: 5+5=FWD  I+M+P=REV  4L=LEFT  4R=RIGHT  L-shape=SERVO")

    while True:
        ret, frame = vs.read()
        if not ret or frame is None:
            continue

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # MediaPipe requires strictly increasing timestamps
        ts_ms = int((time.time() - start_time) * 1000)
        if ts_ms <= vs.last_ts_ms:
            ts_ms = vs.last_ts_ms + 1
        vs.last_ts_ms = ts_ms

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = hands.detect_for_video(mp_image, ts_ms)

        left_fingers = right_fingers = None
        h, w, _ = frame.shape

        if res.hand_landmarks:
            for idx, landmarks in enumerate(res.hand_landmarks):
                # Draw skeleton
                for conn in mp_vision.HandLandmarksConnections.HAND_CONNECTIONS:
                    x1 = int(landmarks[conn.start].x * w)
                    y1 = int(landmarks[conn.start].y * h)
                    x2 = int(landmarks[conn.end].x * w)
                    y2 = int(landmarks[conn.end].y * h)
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                for lm in landmarks:
                    cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 4, (0, 0, 255), -1)

                # MediaPipe returns mirrored labels — flip them
                mp_label = res.handedness[idx][0].category_name
                actual_hand = "Right" if mp_label == "Left" else "Left"

                if actual_hand == "Left" and left_fingers is None:
                    left_fingers = detect_fingers(landmarks, "Left")
                elif actual_hand == "Right" and right_fingers is None:
                    right_fingers = detect_fingers(landmarks, "Right")

        cmd, label, arm_debounce_time = map_gestures(left_fingers, right_fingers, arm_debounce_time)

        # Send UDP at 30 Hz
        now = time.time()
        if now - last_udp_time > CONFIG["FPS_CAP_DELAY"]:
            try:
                tx_sock.sendto(cmd.encode(), (CONFIG["ESP32_IP"], CONFIG["UDP_PORT"]))
            except (socket.error, OSError):
                pass
            last_udp_time = now

        # HUD
        left_str  = "+".join(k[0].upper() for k, v in left_fingers.items()  if v) if left_fingers  else "—"
        right_str = "+".join(k[0].upper() for k, v in right_fingers.items() if v) if right_fingers else "—"
        cv2.putText(frame, f"CMD: {label}",                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"L:[{left_str}]  R:[{right_str}]", (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("Manual Control", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    tx_sock.sendto(b'S', (CONFIG["ESP32_IP"], CONFIG["UDP_PORT"]))
    vs.stop()
    tx_sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
