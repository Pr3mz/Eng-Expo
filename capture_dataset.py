import cv2
import os
import glob
import time

def main():
    # 1. Directory Management
    dataset_dir = os.path.join(os.getcwd(), "dataset", "raw_images")
    os.makedirs(dataset_dir, exist_ok=True)
    
    # Smart File Naming: Find the highest index to prevent overwriting
    existing_files = glob.glob(os.path.join(dataset_dir, "arena_*.jpg"))
    image_counter = 0
    if existing_files:
        indices = []
        for f in existing_files:
            try:
                # Extract the number from 'arena_XXXX.jpg'
                basename = os.path.basename(f)
                num_str = basename.replace("arena_", "").replace(".jpg", "")
                indices.append(int(num_str))
            except ValueError:
                pass
        if indices:
            image_counter = max(indices) + 1
            
    print(f"Directory ready: {dataset_dir}")
    print(f"Starting image index at: {image_counter}")

    # 2. Video Stream Initialization (External Camera = Index 1)
    cap = cv2.VideoCapture(0    )
    
    # Request High Resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    if not cap.isOpened():
        print("Error: Could not open camera index 1. Please check connection.")
        return

    # 3. State Variables
    auto_capture = False
    last_capture_time = 0
    feedback_end_time = 0
    
    print("\n" + "="*30)
    print("      CONTROLS      ")
    print("="*30)
    print("[SPACE] or [s] : Manual Capture")
    print("[a]            : Toggle Auto-Capture Burst Mode (0.5s)")
    print("[q]            : Quit")
    print("==============================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Exiting...")
            break
            
        current_time = time.time()
        save_triggered = False

        # Key checks
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Exiting capture tool.")
            break
        elif key == ord('a'):
            auto_capture = not auto_capture
            print(f"Auto-Capture Mode: {'ON' if auto_capture else 'OFF'}")
        elif key == ord('s') or key == ord(' '):
            save_triggered = True

        # Auto-capture logic
        if auto_capture and (current_time - last_capture_time >= 0.5):
            save_triggered = True

        # Save image logic
        if save_triggered:
            filename = os.path.join(dataset_dir, f"arena_{image_counter:04d}.jpg")
            # Save the clean frame (without text overlays)
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename}")
            image_counter += 1
            last_capture_time = current_time
            feedback_end_time = current_time + 0.5  # Show feedback for 0.5s

        # Visual Feedback Overlay
        display_frame = frame.copy()
        
        # Mode indicator
        mode_text = "MODE: AUTO (0.5s)" if auto_capture else "MODE: MANUAL"
        color = (0, 165, 255) if auto_capture else (255, 255, 0) # Orange vs Cyan
        cv2.putText(display_frame, mode_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        # Saved indicator
        if current_time < feedback_end_time:
            cv2.putText(display_frame, "IMAGE SAVED!", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        cv2.imshow("Dataset Capture Tool", display_frame)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
