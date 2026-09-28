import cv2
import mediapipe as mp
import socket
import time
import threading

# ==========================================
#               CONFIGURATION
# ==========================================
CONFIG = {
    "ESP32_IP": "10.218.230.31",
    "UDP_PORT": 4210,
    "CAMERA_INDEX": 0,
    "FPS_CAP_DELAY": 0.033,      # ~30 FPS UDP send rate
    "ARM_DEBOUNCE_SEC": 1.0,     # Servo toggle cooldown
    "MP_CONFIDENCE": 0.8,
    "HOLD_DELAY_SEC": 0.15,       # Must hold gesture for 300ms before executing
}
# ==========================================


# ==========================================
#          THREADED CAMERA CAPTURE
# ==========================================
class VideoStream:
    """Dedicated thread so cap.read() never blocks the main loop."""
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.last_ts_ms = 0

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        return self

    def _update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        self.cap.release()


# ------------------------------------------
#           GESTURE LOGIC
# ------------------------------------------
def detect_fingers(hand_landmarks, hand_label):
    """Returns a dict of which fingers are up."""
    fingers = {}
    if hand_label == "Right":
        fingers["thumb"] = hand_landmarks[4].x < hand_landmarks[3].x
    else:
        fingers["thumb"] = hand_landmarks[4].x > hand_landmarks[3].x

    fingers["index"]  = hand_landmarks[8].y  < hand_landmarks[6].y
    fingers["middle"] = hand_landmarks[12].y < hand_landmarks[10].y
    fingers["ring"]   = hand_landmarks[16].y < hand_landmarks[14].y
    fingers["pinky"]  = hand_landmarks[20].y < hand_landmarks[18].y
    return fingers


def is_L_shape(f):
    """Thumb + Index up, others down."""
    return f["thumb"] and f["index"] and not f["middle"] and not f["ring"] and not f["pinky"]

def is_imp_combo(f):
    """Index + Middle + Pinky up (ring down)."""
    return f["index"] and f["middle"] and f["pinky"] and not f["ring"]

def is_four_fingers(f):
    """4 fingers up (index+middle+ring+pinky), thumb down."""
    return not f["thumb"] and f["index"] and f["middle"] and f["ring"] and f["pinky"]

def is_five(f):
    """All 5 fingers up."""
    return all(f.values())

def fingers_str(f):
    """Pretty-print which fingers are up."""
    if f is None:
        return ""
    up = [k[0].upper() for k, v in f.items() if v]
    return "+".join(up) if up else "Fist"


def map_gestures(left_fingers, right_fingers, last_arm_time):
    """Map detected fingers to a single-character UDP command."""
    cmd, label_text = 'S', "STOP"
    new_arm_time = last_arm_time

    left_active  = left_fingers is not None
    right_active = right_fingers is not None

    # --- TWO-HAND GESTURES (highest priority) ---
    if left_active and right_active:
        if is_five(left_fingers) and is_five(right_fingers):
            cmd, label_text = 'F', "FORWARD (5+5)"

        elif is_imp_combo(left_fingers) and is_imp_combo(right_fingers):
            cmd, label_text = 'B', "REVERSE (I+M+P)"

        else:
            if is_four_fingers(left_fingers):
                cmd, label_text = 'L', "LEFT PIVOT (4 fingers L)"
            elif is_four_fingers(right_fingers):
                cmd, label_text = 'R', "RIGHT PIVOT (4 fingers R)"
            elif is_L_shape(left_fingers):
                if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                    cmd, label_text = 'O', "SERVO 19 CLOSE (L-shape L)"
                    new_arm_time = time.time()
            elif is_L_shape(right_fingers):
                if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                    cmd, label_text = 'C', "SERVO 19 OPEN (L-shape R)"
                    new_arm_time = time.time()

    # --- SINGLE-HAND GESTURES ---
    elif left_active:
        if is_five(left_fingers):
            cmd, label_text = 'S', "STOP (5 fingers)"
        elif is_four_fingers(left_fingers):
            cmd, label_text = 'L', "LEFT PIVOT (4 fingers L)"
        elif is_L_shape(left_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label_text = 'O', "SERVO 19 CLOSE (L-shape L)"
                new_arm_time = time.time()

    elif right_active:
        if is_five(right_fingers):
            cmd, label_text = 'S', "STOP (5 fingers)"
        elif is_four_fingers(right_fingers):
            cmd, label_text = 'R', "RIGHT PIVOT (4 fingers R)"
        elif is_L_shape(right_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label_text = 'C', "SERVO 19 OPEN (L-shape R)"
                new_arm_time = time.time()

    return cmd, label_text, new_arm_time


# ------------------------------------------
#           MAIN LOOP
# ------------------------------------------
def main():
    # UDP Socket (non-blocking)
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setblocking(False)

    # MediaPipe Hand Landmarker
    mp_hands = mp.tasks.vision
    BaseOptions = mp.tasks.BaseOptions

    options = mp_hands.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
        running_mode=mp_hands.RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=CONFIG["MP_CONFIDENCE"],
        min_hand_presence_confidence=CONFIG["MP_CONFIDENCE"],
        min_tracking_confidence=CONFIG["MP_CONFIDENCE"]
    )
    hands = mp_hands.HandLandmarker.create_from_options(options)

    print("Starting Camera...")
    vs = VideoStream(src=CONFIG["CAMERA_INDEX"]).start()
    time.sleep(1.0)

    # State
    arm_debounce_time = 0
    last_udp_time = 0
    servo_closed = False            # Track servo 19 state for HUD badge
    start_time = time.time()

    # --- HOLD-DELAY state ---
    pending_cmd = 'S'               # The command being "charged up"
    pending_label = "STOP"
    pending_since = 0.0             # When this command first appeared
    confirmed_cmd = 'S'             # Last command that passed the hold threshold
    confirmed_label = "STOP"

    print("UDP Telemetry Active. Hold-Delay Mode.")
    print("CONTROLS: Press 'q' to quit.\n")

    while True:
        ret, frame = vs.read()
        if not ret or frame is None:
            continue

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        current_ms = int((time.time() - start_time) * 1000)
        if current_ms <= vs.last_ts_ms:
            current_ms = vs.last_ts_ms + 1
        vs.last_ts_ms = current_ms

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = hands.detect_for_video(mp_image, current_ms)

        # --- Detect hands ---
        left_fingers, right_fingers = None, None

        if res.hand_landmarks:
            for idx, hand_landmarks in enumerate(res.hand_landmarks):
                h, w, _ = frame.shape
                # Draw skeleton
                for conn in mp.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS:
                    s, e = conn.start, conn.end
                    x1, y1 = int(hand_landmarks[s].x * w), int(hand_landmarks[s].y * h)
                    x2, y2 = int(hand_landmarks[e].x * w), int(hand_landmarks[e].y * h)
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                for mark in hand_landmarks:
                    x, y = int(mark.x * w), int(mark.y * h)
                    cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)

                mp_label = res.handedness[idx][0].category_name
                actual_hand = "Right" if mp_label == "Left" else "Left"

                if actual_hand == "Left" and left_fingers is None:
                    left_fingers = detect_fingers(hand_landmarks, "Left")
                elif actual_hand == "Right" and right_fingers is None:
                    right_fingers = detect_fingers(hand_landmarks, "Right")

        # --- Map gesture ---
        raw_cmd, raw_label, arm_debounce_time = map_gestures(
            left_fingers, right_fingers, arm_debounce_time
        )

        # --- HOLD-DELAY FILTER ---
        # The gesture must be held steady for HOLD_DELAY_SEC before it fires.
        # Servo commands (O/C) bypass the hold delay (they have their own debounce).
        now = time.time()

        if raw_cmd in ('O', 'C'):
            # Servo commands fire immediately (debounce is handled inside map_gestures)
            confirmed_cmd = raw_cmd
            confirmed_label = raw_label
            pending_cmd = 'S'
            pending_since = now
        elif raw_cmd == pending_cmd:
            # Same gesture held — check if it has been held long enough
            if now - pending_since >= CONFIG["HOLD_DELAY_SEC"]:
                confirmed_cmd = raw_cmd
                confirmed_label = raw_label
        else:
            # Gesture changed — start a new hold timer
            pending_cmd = raw_cmd
            pending_label = raw_label
            pending_since = now

        # Calculate hold progress bar (0.0 to 1.0)
        if raw_cmd == pending_cmd and confirmed_cmd != raw_cmd and raw_cmd not in ('O', 'C'):
            hold_progress = min(1.0, (now - pending_since) / CONFIG["HOLD_DELAY_SEC"])
        else:
            hold_progress = 1.0

        # --- Send UDP ---
        if now - last_udp_time > CONFIG["FPS_CAP_DELAY"]:
            try:
                tx_sock.sendto(confirmed_cmd.encode(), (CONFIG["ESP32_IP"], CONFIG["UDP_PORT"]))
            except (socket.error, OSError):
                pass
            last_udp_time = now

            # Track servo state for HUD
            if confirmed_cmd == 'C':
                servo_closed = True
            elif confirmed_cmd == 'O':
                servo_closed = False

        # --- Draw HUD ---
        left_str = fingers_str(left_fingers)
        right_str = fingers_str(right_fingers)

        cv2.putText(frame, f"Active: {confirmed_label}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"L: [{left_str}] | R: [{right_str}]", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Hold progress bar (only when charging up a new command)
        if hold_progress < 1.0:
            bar_w = int(200 * hold_progress)
            cv2.rectangle(frame, (10, 85), (210, 105), (80, 80, 80), -1)
            cv2.rectangle(frame, (10, 85), (10 + bar_w, 105), (0, 200, 255), -1)
            cv2.putText(frame, f"Hold: {raw_label}", (10, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        # Servo 19 status badge
        servo_text = "GPIO 19: OPEN" if servo_closed else "GPIO 19: CLOSE"
        servo_color = (0, 0, 255) if servo_closed else (0, 200, 0)
        badge_y = 140 if hold_progress < 1.0 else 90
        cv2.rectangle(frame, (10, badge_y), (260, badge_y + 32), servo_color, -1)
        cv2.putText(frame, servo_text, (18, badge_y + 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv2.imshow("CPE Manual Control HUD", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Exiting...")
            break

    vs.stop()
    tx_sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()