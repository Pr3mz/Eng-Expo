import cv2
import mediapipe as mp
import socket
import time
import threading

ESP32_IP = "192.168.1.128" # Update to your ESP32 IP
UDP_PORT = 4210
TELEMETRY_PORT = 4211

# Initialize lightning-fast UDP Socket (non-blocking) for SENDING
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

# Initialize UDP Socket for RECEIVING telemetry
rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.bind(("0.0.0.0", TELEMETRY_PORT))
rx_sock.setblocking(False)

# On Windows, suppress WSAECONNRESET (10054) caused by ICMP port unreachable messages
if hasattr(socket, 'SIO_UDP_CONNRESET'):
    try:
        sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        rx_sock.ioctl(socket.SIO_UDP_CONNRESET, False)
    except Exception:
        pass

# Global variable to store HuskyLens telemetry
huskylens_data = "Waiting for Robot..."

def telemetry_listener():
    global huskylens_data
    while True:
        try:
            data, addr = rx_sock.recvfrom(1024)
            msg = data.decode('utf-8', errors='ignore').strip()
            if msg.startswith("HL|"):
                huskylens_data = msg.replace("HL|", "")
        except (BlockingIOError, socket.error):
            time.sleep(0.01) # Sleep briefly to prevent 100% CPU usage
        except Exception:
            time.sleep(0.01)

# Start telemetry listener thread
threading.Thread(target=telemetry_listener, daemon=True).start()

# Async Camera Thread (Keeps FPS high)
class VideoStream:
    def __init__(self, src=0):
        self.cap = None
        # Try DSHOW first (most reliable for Windows USB/PC webcams), then MSMF, then default
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        indices = [src] if src != 0 else [0, ]

        for backend in backends:
            for idx in indices:
                cap = cv2.VideoCapture(idx, backend)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    for _ in range(3):  # Let camera sensor auto-expose
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            self.cap = cap
                            print(f"Camera opened successfully on Index {idx} (backend: {backend})")
                            break
                    if self.cap is not None:
                        break
                cap.release()
            if self.cap is not None:
                break

        if self.cap is None:
            err_msg = (
                "\n" + "="*70 + "\n"
                "ERROR: Unable to open PC Camera!\n\n"
                "Cause: OpenCV 5.0.0 preview build has a known bug on Windows where\n"
                "VideoCapture cannot initialize camera devices.\n\n"
                "Fix by running this command in your PowerShell terminal:\n"
                "  & .\\.venv-py311\\Scripts\\python.exe -m pip uninstall -y opencv-python opencv-contrib-python\n"
                "  & .\\.venv-py311\\Scripts\\python.exe -m pip install \"opencv-contrib-python<5.0\"\n"
                + "="*70
            )
            raise RuntimeError(err_msg)

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
vs = VideoStream(src=0).start()
time.sleep(1.0) 

arm_debounce_time = 0
last_udp_time = 0
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
            # Send toggle command once, then send STOP during debounce
            if time.time() - arm_debounce_time > 1.0:
                cmd, label_text = 'Q', "TOGGLE LEFT ARM"
                arm_debounce_time = time.time()
            else:
                cmd, label_text = 'S', "STOP (L-Arm Debounce)"
                
        elif right_count == 2:
            if time.time() - arm_debounce_time > 1.0:
                cmd, label_text = 'E', "TOGGLE RIGHT ARM"
                arm_debounce_time = time.time()
            else:
                cmd, label_text = 'S', "STOP (R-Arm Debounce)"

    # --- SEND UDP PACKET ---
    # We send the command continuously so the ESP32 watchdog doesn't trigger.
    # Handled via try-except so dropped packets or full buffers never stall the OpenCV loop.
    if time.time() - last_udp_time > 0.033:  # Cap at ~30 FPS
        try:
            sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
        except (socket.error, OSError):
            pass  # Real-time telemetry: safely drop packet and continue without blocking
        last_udp_time = time.time()
        
    last_cmd = cmd

    # --- DRAW HUD ---
    cv2.putText(frame, f"Command: {label_text}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(frame, f"L-Fingers: {left_count} | R-Fingers: {right_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # Overlay HuskyLens Robot Telemetry
    cv2.putText(frame, f"Robot Vision: {huskylens_data}", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    cv2.imshow("CPE UDP HUD", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vs.stop()
sock.close()
rx_sock.close()
cv2.destroyAllWindows()