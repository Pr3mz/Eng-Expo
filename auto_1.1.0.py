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
ESP32_IP = "192.168.1.128" # Replace with actual IP shown on TFT screen
UDP_PORT = 4210
CAMERA_INDEX = 1

WARP_W = 1000
WARP_H = 750
MM_PER_PIXEL = 2.0  # Adjust based on real-world dimensions (e.g., 2000mm arena / 1000 pixels)
PROXIMITY_THRESHOLD_MM = 150 # 15cm trigger distance

# Roboflow Configuration
RF_API_KEY = "X5wKaATp2EknpnzoIAqF"
RF_WORKSPACE = "prem-supthaksina"
RF_WORKFLOW = "arena-gem-and-drop-zone-detectio"

# ==========================================
#               GLOBALS
# ==========================================
calibration_points = []
homography_matrix = None
latest_warped_frame = None

roboflow_target = None  # Tuple (x, y)
roboflow_active = True

# UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==========================================
#               HELPERS
# ==========================================
def send_udp(cmd):
    try:
        sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
    except Exception:
        pass

def extract_roboflow_target(data):
    if isinstance(data, dict):
        if 'x' in data and 'y' in data:
            return (int(data['x']), int(data['y']))
        if 'center_x' in data and 'center_y' in data:
            return (int(data['center_x']), int(data['center_y']))
        for k, v in data.items():
            res = extract_roboflow_target(v)
            if res: return res
    elif isinstance(data, list):
        for item in data:
            res = extract_roboflow_target(item)
            if res: return res
    return None

# ==========================================
#           ROBOFLOW BACKGROUND THREAD
# ==========================================
def roboflow_thread():
    global roboflow_target
    client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",
        api_key=RF_API_KEY
    ).configure(InferenceConfiguration(api_key_transport="header"))
    
    print("[Roboflow Thread] Started!")
    while roboflow_active:
        if latest_warped_frame is not None:
            try:
                result = client.run_workflow(
                    workspace_name=RF_WORKSPACE,
                    workflow_id=RF_WORKFLOW,
                    images={"image": latest_warped_frame},
                    use_cache=True
                )
                coords = extract_roboflow_target(result)
                if coords:
                    roboflow_target = coords
                    print(f"[Roboflow Thread] Found Target at: {coords}")
            except Exception as e:
                print(f"[Roboflow Thread] Error: {e}")
        time.sleep(0.5)

# ==========================================
#           MOUSE CALLBACK
# ==========================================
def mouse_callback(event, x, y, flags, param):
    global calibration_points, homography_matrix
    if event == cv2.EVENT_LBUTTONDOWN and len(calibration_points) < 4:
        calibration_points.append((x, y))
        print(f"Calibration Point {len(calibration_points)}: ({x}, {y})")
        if len(calibration_points) == 4:
            pts_src = np.array(calibration_points, dtype=np.float32)
            pts_dst = np.array([
                [0, 0],
                [WARP_W, 0],
                [WARP_W, WARP_H],
                [0, WARP_H]
            ], dtype=np.float32)
            homography_matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
            print(">>> Homography Matrix Computed! Switching to AI Navigation... <<<")

# ==========================================
#               MAIN
# ==========================================
def main():
    global latest_warped_frame, roboflow_active, homography_matrix, calibration_points
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    cv2.namedWindow("Arena")
    cv2.setMouseCallback("Arena", mouse_callback)
    
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    
    threading.Thread(target=roboflow_thread, daemon=True).start()
    
    print("\n--- INSTRUCTIONS ---")
    print("1. Click the 4 corners in this order: Top-Left, Top-Right, Bottom-Right, Bottom-Left")
    print("2. The system will auto-warp to 1000x750 and start steering the robot.")
    print("3. Press 'r' to reset calibration.")
    print("4. Press 'q' to quit.\n")

    state = "SEEKING_GEM"
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        display_frame = frame.copy()
        
        # ---------------------------------------------
        # STATE 1: HOMOGRAPHY CALIBRATION
        # ---------------------------------------------
        if homography_matrix is None:
            # Draw crosshairs and labels for clicked points
            labels = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
            for i, pt in enumerate(calibration_points):
                # Crosshair
                cv2.line(display_frame, (pt[0]-15, pt[1]), (pt[0]+15, pt[1]), (0, 255, 0), 2)
                cv2.line(display_frame, (pt[0], pt[1]-15), (pt[0], pt[1]+15), (0, 255, 0), 2)
                # Label
                cv2.putText(display_frame, f"{i+1}. {labels[i]}", (pt[0]+10, pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
            cv2.putText(display_frame, f"Click 4 inner corners ({len(calibration_points)}/4) -> {labels[len(calibration_points)] if len(calibration_points) < 4 else 'Done'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Arena", display_frame)
            
        # ---------------------------------------------
        # STATE 2: ACTIVE NAVIGATION
        # ---------------------------------------------
        else:
            warped = cv2.warpPerspective(frame, homography_matrix, (WARP_W, WARP_H))
            
            # Update the global frame so the Roboflow background thread can process it
            latest_warped_frame = warped.copy()
            
            # 1. ArUco Robot Tracking
            corners, ids, _ = detector.detectMarkers(warped)
            robot_pos = None
            robot_heading = 0.0
            
            if ids is not None and 30 in ids:
                idx = np.where(ids == 30)[0][0]
                marker_corners = corners[idx][0]
                
                # Centroid
                cx = int(np.mean(marker_corners[:, 0]))
                cy = int(np.mean(marker_corners[:, 1]))
                robot_pos = (cx, cy)
                
                # Heading Vector (Assumes forward is between the two top marker corners)
                front_x = (marker_corners[0][0] + marker_corners[1][0]) / 2.0
                front_y = (marker_corners[0][1] + marker_corners[1][1]) / 2.0
                
                cv2.circle(warped, (cx, cy), 5, (255, 0, 0), -1)
                cv2.line(warped, (cx, cy), (int(front_x), int(front_y)), (0, 255, 0), 3)
                
                robot_heading = math.atan2(front_y - cy, front_x - cx)
            
            # 2. Draw Roboflow AI Target
            if roboflow_target is not None:
                cv2.circle(warped, roboflow_target, 8, (0, 255, 255), -1)
                cv2.putText(warped, "AI TARGET", (roboflow_target[0]+10, roboflow_target[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
            # 3. Closed-Loop Steering Math
            if robot_pos and roboflow_target and state == "SEEKING_GEM":
                dx = roboflow_target[0] - robot_pos[0]
                dy = roboflow_target[1] - robot_pos[1]
                distance_px = math.hypot(dx, dy)
                distance_mm = distance_px * MM_PER_PIXEL
                
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - robot_heading
                
                # Normalize angle diff to [-PI, PI]
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
                
                cv2.putText(warped, f"Distance: {int(distance_mm)}mm | Heading Error: {angle_diff:.2f} rad", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # TERMINAL ACQUISITION
                if distance_mm < PROXIMITY_THRESHOLD_MM:
                    print(">>> PROXIMITY TRIGGERED! GRABBING PAYLOAD! <<<")
                    send_udp('S') # Stop motors
                    time.sleep(0.1)
                    send_udp('C') # Close servos
                    state = "GRABBED"
                
                # GLOBAL STEERING
                else:
                    # Very simple proportional steering
                    if angle_diff > 0.3:
                        send_udp('R')
                    elif angle_diff < -0.3:
                        send_udp('L')
                    else:
                        send_udp('F')
            else:
                send_udp('S') # Stop if we lose tracking
                if state == "GRABBED":
                    cv2.putText(warped, "PAYLOAD SECURED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
            cv2.imshow("Arena", warped)
            
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            print(">>> Resetting Calibration <<<")
            homography_matrix = None
            calibration_points = []

    # Cleanup
    roboflow_active = False
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
