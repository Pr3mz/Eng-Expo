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
def count_fingers(hand_landmarks, hand_label):
    fingers = 0
    # Thumb
    if hand_label == "Right":
        if hand_landmarks[4].x < hand_landmarks[3].x: fingers += 1
    else:
        if hand_landmarks[4].x > hand_landmarks[3].x: fingers += 1
        
    # Index, Middle, Ring, Pinky
    for tip in [8, 12, 16, 20]:
        if hand_landmarks[tip].y < hand_landmarks[tip - 2].y:
            fingers += 1
            
    return fingers


def map_gestures(left_count, right_count, last_arm_time):
    cmd, label_text = 'S', "STOP"
    new_arm_time = last_arm_time
    
    # 1. Dual-Hand Drive Commands
    if left_count == 5 and right_count == 5:
        cmd, label_text = 'F', "FORWARD (5 + 5)"
    elif left_count == 3 and right_count == 3:
        cmd, label_text = 'B', "REVERSE (3 + 3)"
    elif left_count == 0 and right_count == 0:
        cmd, label_text = 'S', "STOP (Fists)"
        
    # 2. Single-Hand Pivot & Arm Commands
    else:
        if left_count == 1:   
            cmd, label_text = 'L', "LEFT PIVOT (Left: 1)"
        elif right_count == 1: 
            cmd, label_text = 'R', "RIGHT PIVOT (Right: 1)"
            
        elif left_count == 2 or right_count == 2:
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label_text = 'G', "SMART GRIP"
                new_arm_time = time.time()
            else:
                cmd, label_text = 'S', "STOP (Grip Debounce)"
                
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
        
        left_count, right_count = 0, 0

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
                
                if actual_hand == "Left":
                    left_count = count_fingers(hand_landmarks, "Left")
                elif actual_hand == "Right":
                    right_count = count_fingers(hand_landmarks, "Right")

        # Map gestures to commands
        cmd, label_text, arm_debounce_time = map_gestures(left_count, right_count, arm_debounce_time)

        # Send UDP Packet
        if time.time() - last_udp_time > CONFIG["FPS_CAP_DELAY"]:
            try:
                tx_sock.sendto(cmd.encode(), (CONFIG["ESP32_IP"], CONFIG["UDP_PORT"]))
            except (socket.error, OSError):
                pass 
            last_udp_time = time.time()

        # Draw HUD
        cv2.putText(frame, f"Command: {label_text}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"L-Fingers: {left_count} | R-Fingers: {right_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

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

