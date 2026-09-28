import cv2
import numpy as np
import socket
import time
import math
import select

# ==========================================
#               CONFIGURATION
# ==========================================
ESP32_IP = "192.168.1.128"
UDP_PORT = 4210
TELEMETRY_PORT = 4211
CAMERA_INDEX = 0

# Arena / Target Settings
# The homography will warp the arena to a 500x500 flat square
ARENA_SIZE = 500
TARGET_X, TARGET_Y = 250, 50 # Example drop-off or target location
HANDOVER_DISTANCE_PX = 60    # ~15cm trigger depending on scale

# ==========================================
#               NETWORK SETUP
# ==========================================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

tlm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tlm_sock.bind(("0.0.0.0", TELEMETRY_PORT))
tlm_sock.setblocking(False)
if hasattr(socket, 'SIO_UDP_CONNRESET'):
    try: tlm_sock.ioctl(socket.SIO_UDP_CONNRESET, False)
    except: pass

# ==========================================
#           VISION ALGORITHMS
# ==========================================
def order_points(pts):
    """ Orders 4 points: top-left, top-right, bottom-right, bottom-left """
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def get_arena_corners(frame):
    """
    Automated Map Recognition:
    Uses Gaussian Blur + Canny Edge Detection to find the largest rectangular 
    contour which represents the field boundaries.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Sort by area to find the largest polygon (the arena)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    for c in contours:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        # If it has 4 corners and is significantly large
        if len(approx) == 4 and cv2.contourArea(approx) > 10000:
            return approx.reshape(4, 2)
    return None

# ==========================================
#           MAIN ROBOT LOOP
# ==========================================
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    homography_matrix = None
    state = "GLOBAL" # GLOBAL tracking or LOCAL terminal guidance

    # ArUco Setup for ID 30, DICT_4X4_50
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print("Looking for arena boundaries...")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1. Automated Homography (Calibration Phase)
        if homography_matrix is None:
            corners = get_arena_corners(frame)
            if corners is not None:
                cv2.drawContours(frame, [corners], -1, (0, 255, 0), 2)
                cv2.putText(frame, "Arena Found - Press 'c' to Calibrate", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('c'):
                    ordered = order_points(corners)
                    dst = np.array([
                        [0, 0], 
                        [ARENA_SIZE, 0], 
                        [ARENA_SIZE, ARENA_SIZE], 
                        [0, ARENA_SIZE]
                    ], dtype="float32")
                    homography_matrix = cv2.getPerspectiveTransform(ordered, dst)
                    print("Homography Calibrated!")
            else:
                cv2.putText(frame, "Searching for Arena Bounds...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow("Overhead", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            continue
        
        # 2. Warp the frame using the calibrated homography matrix
        warped = cv2.warpPerspective(frame, homography_matrix, (ARENA_SIZE, ARENA_SIZE))
        
        # 3. Check for ESP32 Telemetry (Handover signal)
        try:
            data, addr = tlm_sock.recvfrom(1024)
            msg = data.decode('utf-8')
            if "GRABBED" in msg:
                state = "GLOBAL"
                print("Received GRABBED signal! Resuming global authority.")
        except (BlockingIOError, socket.error):
            pass
            
        # 4. State Machine Logic
        if state == "GLOBAL":
            # Detect ArUco Marker
            corners, ids, rejected = detector.detectMarkers(warped)
            if ids is not None and 30 in ids:
                idx = np.where(ids == 30)[0][0]
                c = corners[idx][0]
                
                # Center of robot
                cx = (c[0][0] + c[2][0]) / 2
                cy = (c[0][1] + c[2][1]) / 2
                
                # Heading vector (front of robot is midpoint of top two corners)
                fx = (c[0][0] + c[1][0]) / 2
                fy = (c[0][1] + c[1][1]) / 2
                robot_heading = math.atan2(fy - cy, fx - cx)
                
                # Draw Visuals
                cv2.circle(warped, (int(cx), int(cy)), 5, (255, 0, 0), -1)
                cv2.line(warped, (int(cx), int(cy)), (int(fx), int(fy)), (0, 255, 255), 2)
                cv2.circle(warped, (TARGET_X, TARGET_Y), 10, (0, 0, 255), -1)
                cv2.line(warped, (int(cx), int(cy)), (TARGET_X, TARGET_Y), (255, 0, 255), 1)
                
                dist = math.hypot(TARGET_X - cx, TARGET_Y - cy)
                
                # Proximity Handover Trigger
                if dist < HANDOVER_DISTANCE_PX:
                    print(f"Distance {dist:.1f} < Threshold. Handing over to HuskyLens!")
                    sock.sendto(b'X', (ESP32_IP, UDP_PORT))
                    state = "LOCAL"
                else:
                    # Calculate angle error to target
                    target_angle = math.atan2(TARGET_Y - cy, TARGET_X - cx)
                    angle_error = target_angle - robot_heading
                    
                    # Normalize between -pi and pi
                    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
                    
                    # UDP Navigation Command
                    cmd = b'S'
                    if abs(angle_error) > 0.3: # Steering deadband
                        cmd = b'R' if angle_error > 0 else b'L'
                    else:
                        cmd = b'F'
                        
                    sock.sendto(cmd, (ESP32_IP, UDP_PORT))
                    cv2.putText(warped, f"DIST: {int(dist)} CMD: {cmd.decode()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        else:
            cv2.putText(warped, "STATE: LOCAL (Waiting for ESP32 GRABBED signal)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        cv2.imshow("Overhead Warped", warped)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
