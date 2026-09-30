import cv2
import time

class GestureController:
    def __init__(self):
        import mediapipe as mp
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.last_arm_time = 0

    def detect_fingers(self, landmarks, hand_side):
        """Return dict of which fingers are up for the given hand using normalized landmarks."""
        TIP = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
        PIP = {"thumb": 3, "index": 7, "middle": 11, "ring": 15, "pinky": 19}

        fingers = {}
        for name, tip_id in TIP.items():
            tip = landmarks.landmark[tip_id]
            pip = landmarks.landmark[PIP[name]]
            if name == "thumb":
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
        results = self.hands.process(rgb)
        
        cmd = None
        new_is_grabbed = is_grabbed
        left_f = None
        right_f = None

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                label = handedness.classification[0].label # Left or Right
                
                # MediaPipe returns mirrored labels by default for front-facing. 
                # If camera is overhead and NOT flipped, we might need to adjust this.
                if label == "Left":
                    left_f = self.detect_fingers(hand_landmarks, "Left")
                else:
                    right_f = self.detect_fingers(hand_landmarks, "Right")

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
