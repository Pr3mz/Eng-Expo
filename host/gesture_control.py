import cv2
import time
import mediapipe as mp
import os

class GestureController:
    def __init__(self):
        self.mp_vision = mp.tasks.vision
        options = self.mp_vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=os.path.join(os.path.dirname(__file__), 'assets', 'hand_landmarker.task')),
            running_mode=self.mp_vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.hands = self.mp_vision.HandLandmarker.create_from_options(options)
        self.last_arm_time = 0
        self.start_time = time.time()
        self.last_ts_ms = -1

    def detect_fingers(self, landmarks, hand_side):
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

    def is_thumb_and_pinky(self, f): return f["thumb"] and not f["index"] and not f["middle"] and not f["ring"] and f["pinky"]
    def is_L_shape(self, f): return f["thumb"] and f["index"] and not f["middle"] and not f["ring"] and not f["pinky"]
    def is_three_fingers(self, f): return not f["thumb"] and f["index"] and f["middle"] and f["ring"] and not f["pinky"]
    def is_four_fingers(self, f): return not f["thumb"] and f["index"] and f["middle"] and f["ring"] and f["pinky"]
    def is_five(self, f): return all(f.values())

    def process_frame(self, frame, is_grabbed):
        """Processes frame, draws hands, and returns (cmd_char, new_is_grabbed) or None"""
        
        # Mirror the frame to match manual 1.3.0 logic
        frame_flipped = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame_flipped, cv2.COLOR_BGR2RGB)
        
        ts_ms = int((time.time() - self.start_time) * 1000)
        if ts_ms <= self.last_ts_ms:
            ts_ms = self.last_ts_ms + 1
        self.last_ts_ms = ts_ms

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = self.hands.detect_for_video(mp_image, ts_ms)
        
        cmd = 'S'
        new_is_grabbed = is_grabbed
        left_f = None
        right_f = None

        h, w, _ = frame_flipped.shape

        if res.hand_landmarks:
            for idx, landmarks in enumerate(res.hand_landmarks):
                # Draw skeleton
                for conn in self.mp_vision.HandLandmarksConnections.HAND_CONNECTIONS:
                    x1 = int(landmarks[conn.start].x * w)
                    y1 = int(landmarks[conn.start].y * h)
                    x2 = int(landmarks[conn.end].x * w)
                    y2 = int(landmarks[conn.end].y * h)
                    cv2.line(frame_flipped, (x1, y1), (x2, y2), (0, 255, 0), 2)
                for lm in landmarks:
                    cv2.circle(frame_flipped, (int(lm.x * w), int(lm.y * h)), 4, (0, 0, 255), -1)

                # MediaPipe returns mirrored labels — flip them back just like 1.3.0
                mp_label = res.handedness[idx][0].category_name
                actual_hand = "Right" if mp_label == "Left" else "Left"

                if actual_hand == "Left" and left_f is None:
                    left_f = self.detect_fingers(landmarks, "Left")
                elif actual_hand == "Right" and right_f is None:
                    right_f = self.detect_fingers(landmarks, "Right")

            L, R = left_f is not None, right_f is not None
            
            # Check toggle servo
            toggle_triggered = False
            if R and self.is_thumb_and_pinky(right_f): toggle_triggered = True

            if toggle_triggered and (time.time() - self.last_arm_time > 1.0):
                new_is_grabbed = not is_grabbed
                self.last_arm_time = time.time()
                # Copy the flipped frame back so it renders correctly
                frame[:] = cv2.flip(frame_flipped, 1)
                return ('C' if new_is_grabbed else 'O', new_is_grabbed)

            # Movement mapping EXACTLY like 1.3.0
            if L and R:
                if self.is_five(left_f) and self.is_five(right_f): cmd = 'F'
                elif self.is_three_fingers(left_f) and self.is_three_fingers(right_f): cmd = 'B'
                elif self.is_four_fingers(left_f): cmd = 'L'
                elif self.is_four_fingers(right_f): cmd = 'R'
            elif L:
                if self.is_five(left_f): cmd = 'S'
                elif self.is_four_fingers(left_f): cmd = 'L'
            elif R:
                if self.is_five(right_f): cmd = 'S'
                elif self.is_four_fingers(right_f): cmd = 'R'
                
        # To show the tracking correctly, we must flip it back to match the original camera orientation
        # (Otherwise the user sees a mirrored window, which might conflict with Final1.py's window)
        frame[:] = cv2.flip(frame_flipped, 1)

        return cmd, new_is_grabbed

