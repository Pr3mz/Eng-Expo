import cv2
import mediapipe as mp
import socket
import time
import threading
from PIL import Image
import io
import os



# ==========================================
#               CONFIGURATION
# ==========================================
CONFIG = {
    "ESP32_IP": "10.218.230.31",
    "UDP_PORT": 4210,
    "TELEMETRY_PORT": 4211,
    "CAMERA_INDEX": 0,
    "FPS_CAP_DELAY": 0.033,  # ~30 FPS
    "ARM_DEBOUNCE_SEC": 1.0,
    "MP_CONFIDENCE": 0.8
}
# ==========================================


class VideoStream:
    def __init__(self, src=0):
        self.cap = None
        backends = []
        if hasattr(cv2, 'CAP_DSHOW'): backends.append(cv2.CAP_DSHOW)
        if hasattr(cv2, 'CAP_MSMF'): backends.append(cv2.CAP_MSMF)
        backends.append(cv2.CAP_ANY)
        indices = [src] if src != 0 else [0, ]

        for backend in backends:
            for idx in indices:
                cap = cv2.VideoCapture(idx, backend)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    for _ in range(3):
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            self.cap = cap
                            break
                    if self.cap is not None: break
                cap.release()
            if self.cap is not None: break

        if self.cap is None:
            raise RuntimeError("ERROR: Unable to open PC Camera!")

        self.ret, self.frame = self.cap.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
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
    """
    Returns a dict of which fingers are up: {thumb, index, middle, ring, pinky}
    """
    fingers = {}
    
    # Thumb: compare tip.x vs IP.x (direction depends on hand)
    if hand_label == "Right":
        fingers["thumb"] = hand_landmarks[4].x < hand_landmarks[3].x
    else:
        fingers["thumb"] = hand_landmarks[4].x > hand_landmarks[3].x
    
    # Index, Middle, Ring, Pinky: tip.y < PIP.y means finger is up
    fingers["index"]  = hand_landmarks[8].y  < hand_landmarks[6].y
    fingers["middle"] = hand_landmarks[12].y < hand_landmarks[10].y
    fingers["ring"]   = hand_landmarks[16].y < hand_landmarks[14].y
    fingers["pinky"]  = hand_landmarks[20].y < hand_landmarks[18].y
    
    return fingers

def fingers_up_count(f):
    """Count total fingers up from a finger dict."""
    return sum(f.values())

def is_thumb_only(f):
    """Only thumb is up."""
    return f["thumb"] and not f["index"] and not f["middle"] and not f["ring"] and not f["pinky"]

def is_L_shape(f):
    """Thumb + Index up, others down."""
    return f["thumb"] and f["index"] and not f["middle"] and not f["ring"] and not f["pinky"]

def is_imp_combo(f):
    """Index + Middle + Pinky up (ring & thumb can be anything)."""
    return f["index"] and f["middle"] and f["pinky"] and not f["ring"]

def is_four_fingers(f):
    """4 fingers up (index+middle+ring+pinky), thumb down."""
    return not f["thumb"] and f["index"] and f["middle"] and f["ring"] and f["pinky"]

def is_five(f):
    """All 5 fingers up."""
    return all(f.values())


def map_gestures(left_fingers, right_fingers, last_arm_time):
    cmd, label_text = 'S', "STOP"
    new_arm_time = last_arm_time
    
    left_active = left_fingers is not None
    right_active = right_fingers is not None
    
    # --- TWO-HAND GESTURES (highest priority) ---
    if left_active and right_active:
        # 5 + 5 → FORWARD
        if is_five(left_fingers) and is_five(right_fingers):
            cmd, label_text = 'F', "FORWARD (5+5)"
        
        # Index+Middle+Pinky on both → REVERSE
        elif is_imp_combo(left_fingers) and is_imp_combo(right_fingers):
            cmd, label_text = 'B', "REVERSE (I+M+P)"
        
        # If only one hand has a gesture, fall through to single-hand
        else:
            # Check left hand single gestures
            if is_four_fingers(left_fingers):
                cmd, label_text = 'L', "LEFT PIVOT (4 fingers L)"
            elif is_four_fingers(right_fingers):
                cmd, label_text = 'R', "RIGHT PIVOT (4 fingers R)"
            elif is_L_shape(left_fingers):
                if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                    cmd, label_text = 'O', "SERVO 19 OPEN (L-shape L)"
                    new_arm_time = time.time()
            elif is_L_shape(right_fingers):
                if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                    cmd, label_text = 'C', "SERVO 19 CLOSE (L-shape R)"
                    new_arm_time = time.time()
    
    # --- SINGLE-HAND GESTURES ---
    elif left_active:
        if is_five(left_fingers):
            cmd, label_text = 'S', "STOP (5 fingers)"
        elif is_four_fingers(left_fingers):
            cmd, label_text = 'L', "LEFT PIVOT (4 fingers L)"
        elif is_L_shape(left_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label_text = 'O', "SERVO 19 OPEN (L-shape L)"
                new_arm_time = time.time()
    
    elif right_active:
        if is_five(right_fingers):
            cmd, label_text = 'S', "STOP (5 fingers)"
        elif is_four_fingers(right_fingers):
            cmd, label_text = 'R', "RIGHT PIVOT (4 fingers R)"
        elif is_L_shape(right_fingers):
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label_text = 'C', "SERVO 19 CLOSE (L-shape R)"
                new_arm_time = time.time()
    
    return cmd, label_text, new_arm_time



# ------------------------------------------
#           MAIN LOOP
# ------------------------------------------
def main():
    # Setup UDP Sender
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setblocking(False)
    if hasattr(socket, 'SIO_UDP_CONNRESET'):
        try: tx_sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except: pass


    
    mp_hands = mp.tasks.vision
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp_hands.HandLandmarker
    HandLandmarkerOptions = mp_hands.HandLandmarkerOptions
    VisionRunningMode = mp_hands.RunningMode

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=CONFIG["MP_CONFIDENCE"],
        min_hand_presence_confidence=CONFIG["MP_CONFIDENCE"],
        min_tracking_confidence=CONFIG["MP_CONFIDENCE"]
    )
    hands = HandLandmarker.create_from_options(options)

    print("Starting Camera...")
    vs = VideoStream(src=CONFIG["CAMERA_INDEX"]).start()
    time.sleep(1.0) 

    arm_debounce_time = 0
    last_udp_time = 0
    print("UDP Telemetry Active. Zero-Lag Mode.")
    print("CONTROLS: Press 'q' to quit.")

    # Time tracking for mediapipe VIDEO mode
    start_time = time.time()

    while True:
        ret, frame = vs.read()
        if not ret or frame is None: continue

        frame = cv2.flip(frame, 1) 
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        current_ms = int((time.time() - start_time) * 1000)
        # Avoid passing same timestamp or going backwards (MediaPipe requires strictly increasing timestamps)
        if hasattr(vs, 'last_ts_ms') and current_ms <= vs.last_ts_ms:
            current_ms = vs.last_ts_ms + 1
        vs.last_ts_ms = current_ms
        
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = hands.detect_for_video(mp_image, current_ms)
        
        left_fingers, right_fingers = None, None

        if res.hand_landmarks:
            for idx, hand_landmarks in enumerate(res.hand_landmarks):
                # Manual landmark drawing
                h, w, _ = frame.shape
                for connection in mp.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS:
                    start_idx, end_idx = connection.start, connection.end
                    x1, y1 = int(hand_landmarks[start_idx].x * w), int(hand_landmarks[start_idx].y * h)
                    x2, y2 = int(hand_landmarks[end_idx].x * w), int(hand_landmarks[end_idx].y * h)
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

        # Map gestures to commands
        cmd, label_text, arm_debounce_time = map_gestures(left_fingers, right_fingers, arm_debounce_time)

        # Draw HUD
        left_str = ""
        if left_fingers:
            up = [k[0].upper() for k, v in left_fingers.items() if v]
            left_str = "+".join(up) if up else "Fist"
        right_str = ""
        if right_fingers:
            up = [k[0].upper() for k, v in right_fingers.items() if v]
            right_str = "+".join(up) if up else "Fist"

        # Send UDP Packet
        if time.time() - last_udp_time > CONFIG["FPS_CAP_DELAY"]:
            try:
                tx_sock.sendto(cmd.encode(), (CONFIG["ESP32_IP"], CONFIG["UDP_PORT"]))
            except (socket.error, OSError):
                pass 
            last_udp_time = time.time()

        # Draw HUD
        cv2.putText(frame, f"Command: {label_text}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"L: [{left_str}] | R: [{right_str}]", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("CPE UDP HUD", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Exiting...")
            break

    vs.stop()
    tx_sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

