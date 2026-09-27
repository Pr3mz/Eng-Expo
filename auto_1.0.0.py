import cv2
import numpy as np
import socket
import time
import math
import os
import threading

# Configuration
ESP32_IP = "192.168.1.128"
UDP_PORT = 4210
TELEMETRY_PORT = 4211
WARP_FILE = "warp_matrix.npy"

# UDP Sockets (Non-blocking)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.bind(("0.0.0.0", TELEMETRY_PORT))
rx_sock.setblocking(False)

if hasattr(socket, 'SIO_UDP_CONNRESET'):
    try:
        sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        rx_sock.ioctl(socket.SIO_UDP_CONNRESET, False)
    except Exception:
        pass

# Globals
state = "GLOBAL_NAV"  # GLOBAL_NAV or LOCAL_SEARCH
calibration_points = []
matrix = None
target_pt = (400, 300) # Default target inside the 800x600 arena
pixels_per_cm = 10 # Adjust this based on physical arena size
last_udp_time = 0
last_cmd = 'S'

# ArUco Setup (OpenCV 4.7+)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Mouse Callback for Field Calibration & Target Setting
def mouse_callback(event, x, y, flags, param):
    global calibration_points, target_pt
    if event == cv2.EVENT_LBUTTONDOWN and matrix is None:
        if len(calibration_points) < 4:
            calibration_points.append([x, y])
    elif event == cv2.EVENT_RBUTTONDOWN and matrix is not None:
        target_pt = (x, y) # Set target gem location manually

cv2.namedWindow("Overhead Camera")
cv2.setMouseCallback("Overhead Camera", mouse_callback)

if os.path.exists(WARP_FILE):
    matrix = np.load(WARP_FILE)

# Camera Thread
class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
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

def send_cmd(cmd):
    global last_udp_time, last_cmd
    if time.time() - last_udp_time > 0.033 or cmd != last_cmd:
        try:
            sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
            last_cmd = cmd
        except (socket.error, OSError):
            pass
        last_udp_time = time.time()

vs = VideoStream().start()
time.sleep(1.0)

print("System Online.")
print("Left click 4 corners to calibrate field. Right click to set target gem.")

while True:
    ret, frame = vs.read()
    if not ret or frame is None:
        continue

    # 1. Listen for Telemetry from ESP32
    try:
        data, _ = rx_sock.recvfrom(1024)
        msg = data.decode('utf-8', errors='ignore').strip()
        if msg == "GRABBED":
            print("Target Grabbed! Returning to Base.")
            state = "GLOBAL_NAV"
            target_pt = (100, 500) # Arbitrary base coordinates
    except (BlockingIOError, socket.error):
        pass

    # 2. Calibration Phase
    if matrix is None:
        for pt in calibration_points:
            cv2.circle(frame, tuple(pt), 5, (0, 255, 0), -1)
        if len(calibration_points) == 4:
            cv2.putText(frame, "Press 's' to save Homography Matrix", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            key = cv2.waitKey(1)
            if key == ord('s'):
                pts1 = np.float32(calibration_points)
                pts2 = np.float32([[0, 0], [800, 0], [800, 600], [0, 600]])
                matrix = cv2.getPerspectiveTransform(pts1, pts2)
                np.save(WARP_FILE, matrix)
                print("Matrix saved.")
        cv2.imshow("Overhead Camera", frame)
        if cv2.waitKey(1) == ord('q'): break
        continue

    # 3. Global Navigation Phase
    warped = cv2.warpPerspective(frame, matrix, (800, 600))
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    cmd = 'S'
    robot_center = None

    if ids is not None and 0 in ids:
        idx = np.where(ids == 0)[0][0]
        pts = corners[idx][0]
        
        cv2.polylines(warped, [np.int32(pts)], True, (0, 255, 0), 2)
        
        # Center of robot marker
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        robot_center = (cx, cy)
        
        # Heading calculation (Assuming top of marker is between pts[0] and pts[1])
        fx = (pts[0][0] + pts[1][0]) / 2.0
        fy = (pts[0][1] + pts[1][1]) / 2.0
        heading = math.atan2(fy - cy, fx - cx)
        
        cv2.line(warped, (cx, cy), (int(fx), int(fy)), (255, 0, 0), 2)
        cv2.circle(warped, robot_center, 4, (0, 255, 255), -1)

        # Coordinate Mapping to Target
        dist_pixels = math.hypot(target_pt[0] - cx, target_pt[1] - cy)
        dist_cm = dist_pixels / pixels_per_cm
        
        cv2.line(warped, robot_center, target_pt, (0, 255, 255), 1)

        target_angle = math.atan2(target_pt[1] - cy, target_pt[0] - cx)
        angle_diff = (target_angle - heading)
        # Normalize between -pi and pi
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # State Machine Logic
        if state == "GLOBAL_NAV":
            if dist_cm < 15.0:
                print("Proximity triggered. Initiating LOCAL_SEARCH.")
                state = "LOCAL_SEARCH"
                cmd = 'X' # Local search trigger command
            else:
                if abs(angle_diff) > 0.35: # ~20 degrees tolerance
                    if angle_diff > 0:
                        cmd = 'R'
                    else:
                        cmd = 'L'
                else:
                    cmd = 'F'
        elif state == "LOCAL_SEARCH":
            cmd = 'X' # Continuously assert local authority

    else:
        # ArUco lost
        if state == "GLOBAL_NAV":
            cmd = 'S'

    send_cmd(cmd)

    # 4. HUD Updates
    cv2.circle(warped, target_pt, 8, (0, 0, 255), -1)
    cv2.putText(warped, "TARGET", (target_pt[0]+10, target_pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(warped, f"STATE: {state} | CMD: {cmd}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    if robot_center:
        cv2.putText(warped, f"Dist: {dist_cm:.1f}cm | Angle Err: {math.degrees(angle_diff):.1f} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Overhead Camera", warped)
    
    if cv2.waitKey(1) == ord('q'):
        break

vs.stop()
sock.close()
rx_sock.close()
cv2.destroyAllWindows()
