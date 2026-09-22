import cv2
import mediapipe as mp
import socket
import time
import threading
import sys

# ==========================================
#               CONFIGURATION
# ==========================================
CONFIG = {
    "ESP32_IP": "192.168.1.128",
    "UDP_PORT": 4210,
    "TELEMETRY_PORT": 4211,
    "CAMERA_INDEX": 0,
    "FPS_CAP_DELAY": 0.033,  # ~30 FPS
    "ARM_DEBOUNCE_SEC": 1.0,
    "MP_CONFIDENCE": 0.7,
}
# ==========================================

# ------------------------------------------
#           TELEMETRY RECEIVER
# ------------------------------------------
class TelemetryReceiver:
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.huskylens_data = "Waiting for Robot..."
        
        if hasattr(socket, 'SIO_UDP_CONNRESET'):
            try:
                self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except Exception:
                pass
                
        threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                msg = data.decode('utf-8', errors='ignore').strip()
                if msg.startswith("HL|"):
                    self.huskylens_data = msg.replace("HL|", "")
            except (BlockingIOError, socket.error):
                time.sleep(0.01)
            except Exception:
                time.sleep(0.01)

# ------------------------------------------
#           CAMERA STREAM
# ------------------------------------------
class VideoStream:
    def __init__(self, src=0):
        self.cap = None
        if sys.platform == "darwin":
            backends = [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
        elif sys.platform == "win32":
            backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        else:
            backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
            
        indices = [src] if src != 0 else [0, 1]

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
        if self.cap is not None:
            self.cap.release()

# ------------------------------------------
#           GESTURE LOGIC
# ------------------------------------------
def count_fingers(hand_landmarks, hand_label):
    fingers = 0
    # Thumb
    if hand_label == "Right":
        if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x: fingers += 1
    else:
        if hand_landmarks.landmark[4].x > hand_landmarks.landmark[3].x: fingers += 1
        
    # Index, Middle, Ring, Pinky
    for tip in [8, 12, 16, 20]:
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
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
            
        elif left_count == 2:
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label_text = 'Q', "TOGGLE LEFT ARM"
                new_arm_time = time.time()
            else:
                cmd, label_text = 'S', "STOP (L-Arm Debounce)"
                
        elif right_count == 2:
            if time.time() - last_arm_time > CONFIG["ARM_DEBOUNCE_SEC"]:
                cmd, label_text = 'E', "TOGGLE RIGHT ARM"
                new_arm_time = time.time()
            else:
                cmd, label_text = 'S', "STOP (R-Arm Debounce)"
                
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

    telemetry = TelemetryReceiver(CONFIG["TELEMETRY_PORT"])
    
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        max_num_hands=2, 
        min_detection_confidence=CONFIG["MP_CONFIDENCE"], 
        min_tracking_confidence=CONFIG["MP_CONFIDENCE"]
    )

    print("Starting Camera...")
    vs = VideoStream(src=CONFIG["CAMERA_INDEX"]).start()
    time.sleep(1.0) 

    arm_debounce_time = 0
    last_udp_time = 0
    print("UDP Telemetry Active. Zero-Lag Mode.")
    print("CONTROLS: Press 'q' to quit.")

    while True:
        ret, frame = vs.read()
        if not ret or frame is None: continue

        frame = cv2.flip(frame, 1) 
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)
        
        left_count, right_count = 0, 0

        if res.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(res.multi_hand_landmarks):
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                mp_label = res.multi_handedness[idx].classification[0].label
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
        cv2.putText(frame, f"Robot Vision: {telemetry.huskylens_data}", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

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
