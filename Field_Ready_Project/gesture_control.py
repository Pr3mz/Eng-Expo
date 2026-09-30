import cv2
import time
import mediapipe as mp
import os

class GestureController:
    def __init__(self):
        self.mp_vision = mp.tasks.vision
        options = self.mp_vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=os.path.join(os.path.dirname(__file__), 'hand_landmarker.task')),
            running_mode=self.mp_vision.RunningMode.IMAGE,
            num_hands=2,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.hands = self.mp_vision.HandLandmarker.create_from_options(options)
        self.last_arm_time = 0

    def detect_fingers(self, landmarks, hand_side):
        """Return dict of which fingers are up for the given hand."""
        TIP = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
        PIP = {"thumb": 3, "index": 7, "middle": 11, "ring": 15, "pinky": 19}

        fingers = {}
        for name, tip_id in TIP.items():
            tip = landmarks[tip_id]
            pip = landmarks[PIP[name]]
            if name == "thumb":
                # Thumb: compare x-axis
                if hand_side == "Right":
                    fingers["thumb"] = tip.x < pip.x
                else:
                    fingers["thumb"] = tip.x > pip.x
            else:
                fingers[name] = tip.y < pip.y
        return fingers

    def is_L_shape(self, f): return f["thumb"] and f["index"] and not f["middle"] and not f["ring"] and not f["pinky"]
    def is_imp_combo(self, f): return f["index"] and f["middle"] and f["pinky"] and not f["ring"]
    def is_four_fingers(self, f): return not f["thumb"] and f["index"] and f["middle"] and f["ring"] and f["pinky"]
    def is_five(self, f): return all(f.values())

    def process_frame(self, frame, is_grabbed):
        """Processes frame, draws hands, and returns (cmd_char, new_is_grabbed) or None"""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = self.hands.detect(mp_image)
        
        cmd = None
        new_is_grabbed = is_grabbed
        left_f = None
        right_f = None

        h, w, _ = frame.shape

        if res.hand_landmarks:
            for idx, landmarks in enumerate(res.hand_landmarks):
                # Draw skeleton
                for conn in self.mp_vision.HandLandmarksConnections.HAND_CONNECTIONS:
                    x1 = int(landmarks[conn.start].x * w)
                    y1 = int(landmarks[conn.start].y * h)
                    x2 = int(landmarks[conn.end].x * w)
                    y2 = int(landmarks[conn.end].y * h)
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                for lm in landmarks:
                    cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 4, (0, 0, 255), -1)

                mp_label = res.handedness[idx][0].category_name
                # Note: overhead camera might not be mirrored like a selfie cam.
                actual_hand = mp_label

                if actual_hand == "Left" and left_f is None:
                    left_f = self.detect_fingers(landmarks, "Left")
                elif actual_hand == "Right" and right_f is None:
                    right_f = self.detect_fingers(landmarks, "Right")

            L, R = left_f is not None, right_f is not None
            
            # Check toggle servo
            toggle_triggered = False
            if L and self.is_L_shape(left_f): toggle_triggered = True
            if R and self.is_L_shape(right_f): toggle_triggered = True

            if toggle_triggered and (time.time() - self.last_arm_time > 1.0):
                new_is_grabbed = not is_grabbed
                self.last_arm_time = time.time()
                return ('C' if new_is_grabbed else 'O', new_is_grabbed)

            # Movement mapping
            if L and R:
                if self.is_five(left_f) and self.is_five(right_f): cmd = 'F'
                elif self.is_imp_combo(left_f) and self.is_imp_combo(right_f): cmd = 'B'
                elif self.is_four_fingers(left_f): cmd = 'L'
                elif self.is_four_fingers(right_f): cmd = 'R'
            elif L:
                if self.is_five(left_f): cmd = 'S'
                elif self.is_four_fingers(left_f): cmd = 'L'
            elif R:
                if self.is_five(right_f): cmd = 'S'
                elif self.is_four_fingers(right_f): cmd = 'R'

        return cmd, new_is_grabbed
