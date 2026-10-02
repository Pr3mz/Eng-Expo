import cv2
import math
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
        self.fist_stop = False
        self.debug_text = ""      # what the camera reads, shown on screen in manual mode
        self.min_hand_span = 0.0   # ignore hands smaller than this (fraction of frame height); set by argos.py
        self._switch_armed = True   # gripper switch needs the gesture released between flips

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
    def is_back_pose(self, f): return f["index"] and f["middle"] and not f["ring"] and not f["pinky"]  # index + middle, thumb ignored
    def is_index_pinky(self, f): return f["index"] and f["pinky"] and not f["middle"] and not f["ring"]  # thumb ignored
    def is_four_fingers(self, f): return not f["thumb"] and f["index"] and f["middle"] and f["ring"] and f["pinky"]
    def is_five(self, f): return all(f.values())
    def is_fist(self, f): return not any(f.values())   # no fingers up

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
        self.fist_stop = False   # True when a closed fist (no fingers) asks for an immediate stop
        new_is_grabbed = is_grabbed
        left_f = None
        right_f = None

        h, w, _ = frame_flipped.shape

        # Keep only the two biggest hands that are big enough: people and hands in
        # the background must not drive the robot. Size = wrist to middle knuckle.
        def span(landmarks):
            return math.hypot((landmarks[9].x - landmarks[0].x) * w, (landmarks[9].y - landmarks[0].y) * h) / h

        candidates = sorted(range(len(res.hand_landmarks)), key=lambda i: -span(res.hand_landmarks[i]))
        kept = [i for i in candidates if span(res.hand_landmarks[i]) >= self.min_hand_span][:2]
        ignored = len(res.hand_landmarks) - len(kept)
        hand_list = [(res.hand_landmarks[i], res.handedness[i][0].category_name) for i in kept]
        # If both hands get the same label, tell them apart by position (mirrored view:
        # the hand on the left of the picture is the left hand).
        sides = ["Right" if label == "Left" else "Left" for _, label in hand_list]
        if len(hand_list) == 2 and sides[0] == sides[1]:
            order = sorted(range(2), key=lambda k: hand_list[k][0][0].x)
            sides[order[0]], sides[order[1]] = "Left", "Right"

        if not hand_list:
            self._switch_armed = True
            self.debug_text = "no hands seen" if not ignored else f"{ignored} small hand(s) ignored (lower Min hand size in G)"
        if hand_list:
            for idx, (landmarks, _label) in enumerate(hand_list):
                # Draw skeleton
                for conn in self.mp_vision.HandLandmarksConnections.HAND_CONNECTIONS:
                    x1 = int(landmarks[conn.start].x * w)
                    y1 = int(landmarks[conn.start].y * h)
                    x2 = int(landmarks[conn.end].x * w)
                    y2 = int(landmarks[conn.end].y * h)
                    cv2.line(frame_flipped, (x1, y1), (x2, y2), (0, 255, 0), 2)
                for lm in landmarks:
                    cv2.circle(frame_flipped, (int(lm.x * w), int(lm.y * h)), 4, (0, 0, 255), -1)

                actual_hand = sides[idx]

                if actual_hand == "Left" and left_f is None:
                    left_f = self.detect_fingers(landmarks, "Left")
                elif actual_hand == "Right" and right_f is None:
                    right_f = self.detect_fingers(landmarks, "Right")

            L, R = left_f is not None, right_f is not None
            names = lambda f: "".join(k for k, n in zip("TIMRP", ("thumb", "index", "middle", "ring", "pinky")) if f[n]) or "fist"
            sizes = "/".join(f"{span(lm) * 100:.0f}%" for lm, _ in hand_list)
            self.debug_text = (f"hands:{len(hand_list)} size {sizes}" + (f" (+{ignored} ignored)" if ignored else "") + f"  L:{names(left_f) if L else '-'}  "
                               f"R:{names(right_f) if R else '-'}  (T I M R P = thumb index middle ring pinky)")
            
            # Gripper works like a switch: index + pinky on EITHER hand flips
            # open <-> closed, and it stays there until the next index + pinky.
            # The gesture must be released before it can flip again, so holding
            # it up does not make the gripper flutter.
            switch_pose = (L and self.is_index_pinky(left_f)) or (R and self.is_index_pinky(right_f))
            toggle_triggered = False
            if switch_pose:
                if self._switch_armed:
                    toggle_triggered = True
                    self._switch_armed = False
            else:
                self._switch_armed = True

            if toggle_triggered and (time.time() - self.last_arm_time > 0.5):
                new_is_grabbed = not is_grabbed
                self.last_arm_time = time.time()
                # Copy the flipped frame back so it renders correctly
                frame[:] = cv2.flip(frame_flipped, 1)
                return ('C' if new_is_grabbed else 'O', new_is_grabbed)

            # Movement mapping EXACTLY like 1.3.0
            if L and R:
                if self.is_five(left_f) and self.is_five(right_f): cmd = 'F'
                elif self.is_back_pose(left_f) and self.is_back_pose(right_f): cmd = 'B'   # index + middle on both hands
                elif self.is_four_fingers(left_f): cmd = 'L'
                elif self.is_four_fingers(right_f): cmd = 'R'
            elif L:
                if self.is_five(left_f): cmd = 'S'
                elif self.is_four_fingers(left_f): cmd = 'L'
            elif R:
                if self.is_five(right_f): cmd = 'S'
                elif self.is_four_fingers(right_f): cmd = 'R'
                
            # A closed fist (no fingers up) with no drive gesture = stop right now.
            if cmd == 'S' and ((L and self.is_fist(left_f)) or (R and self.is_fist(right_f))):
                self.fist_stop = True

        # To show the tracking correctly, we must flip it back to match the original camera orientation
        # (Otherwise the user sees a mirrored window, which might conflict with Final1.py's window)
        frame[:] = cv2.flip(frame_flipped, 1)
        if self.debug_text:
            self.debug_text += f"  -> {cmd}"

        return cmd, new_is_grabbed

