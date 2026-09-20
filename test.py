import cv2
import mediapipe as mp
import socket
import time
import threading

ESP32_IP = "192.168.1.128" # Update to your ESP32 IP
UDP_PORT = 4210

# Initialize lightning-fast UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Async Camera Thread (Keeps FPS high)
class VideoStream:
    def __init__(self, src=1):
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
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

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.7)

print("Starting Camera...")
vs = VideoStream(src=1).start()
time.sleep(1.0) 

arm_debounce_time = 0
last_cmd = 'S'
print("UDP Telemetry Active. Zero-Lag Mode.")

while True:
    ret, frame = vs.read()
    if not ret or frame is None: continue

    frame = cv2.flip(frame, 1) 
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    res = hands.process(rgb)
    
    left_count, right_count = 0, 0
    cmd, label_text = 'S', "STOP"

    if res.multi_hand_landmarks:
        for idx, hand_landmarks in enumerate(res.multi_hand_landmarks):
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            mp_label = res.multi_handedness[idx].classification[0].label
            actual_hand = "Right" if mp_label == "Left" else "Left"
            
            if actual_hand == "Left":
                left_count = count_fingers(hand_landmarks, "Left")
            elif actual_hand == "Right":
                right_count = count_fingers(hand_landmarks, "Right")

    # --- 1-to-5 FINGER DICTIONARY ---
    
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
            # Prevent the servo from rapidly opening and closing instantly
            if time.time() - arm_debounce_time > 0.8:
                cmd, label_text = 'Q', "TOGGLE LEFT ARM"
                arm_debounce_time = time.time()
            else:
                cmd = last_cmd # Hold previous state while debouncing
                
        elif right_count == 2:
            if time.time() - arm_debounce_time > 0.8:
                cmd, label_text = 'E', "TOGGLE RIGHT ARM"
                arm_debounce_time = time.time()
            else:
                cmd = last_cmd

    # --- SEND UDP PACKET ---
    # We send the command continuously so the ESP32 watchdog doesn't trigger
    sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
    last_cmd = cmd

    # --- DRAW HUD ---
    cv2.putText(frame, f"Command: {label_text}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(frame, f"L-Fingers: {left_count} | R-Fingers: {right_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.imshow("CPE UDP HUD", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vs.stop()
cv2.destroyAllWindows()