import cv2
import sys
import os
import random
import glob

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "host"))
from vision import detect_gems_roboflow

def run_dataset_test():
    print("🧠 [START] AI Vision (Roboflow) Test - Random Dataset Image")
    
    # 1. Get images
    dataset_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data", "dataset", "raw_images", "*.jpg"))
    images = glob.glob(dataset_path)
    
    if not images:
        print("❌ ไม่พบรูปใน data/dataset/raw_images/")
        return
        
    img_path = random.choice(images)
    print(f"📸 สุ่มได้รูป: {os.path.basename(img_path)}")
    
    img = cv2.imread(img_path)
    if img is None:
        print("❌ อ่านไฟล์รูปไม่สำเร็จ")
        return
        
    print(f"📸 ขนาดรูป: {img.shape}")
    print("☁️ กำลังส่งขึ้น Roboflow Cloud...")
    
    try:
        # 2. Run inference
        gems, drops = detect_gems_roboflow(img, warp_w=800, warp_h=600)
        
        print(f"\n✅ ประมวลผลเสร็จสิ้น!")
        print(f"💎 พบเพชร: {len(gems)} เม็ด")
        for i, g in enumerate(gems):
            print(f"   [{i+1}] {g.color:7s} | ({g.x:3d}, {g.y:3d})")
            
        print(f"🎯 พบฐาน: {len(drops)} ฐาน")
        for i, d in enumerate(drops):
            print(f"   [{i+1}] {d[2]:7s} | ({d[0]:3d}, {d[1]:3d})")
            
        # 3. Draw annotations
        # Resize image for drawing (since vision.py resizes it to 800x600 internally for detection)
        # Wait! detect_gems_roboflow maps coordinates BACK to the original image dimensions.
        # But we pass the original image.
        
        for g in gems:
            cv2.circle(img, (int(g.x), int(g.y)), 20, (0, 255, 255), 4)
            cv2.putText(img, g.color, (int(g.x) + 20, int(g.y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
            
        for d in drops:
            cv2.drawMarker(img, (int(d[0]), int(d[1])), (255, 0, 255), cv2.MARKER_SQUARE, 30, 4)
            cv2.putText(img, d[2], (int(d[0]) + 20, int(d[1]) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 3)
            
        # 4. Save to artifacts
        out_path = "/Users/prem/.gemini/antigravity/brain/27c81dd2-ad5f-4b29-b63e-ca907ad9cdd4/ai_test_dataset.jpg"
        cv2.imwrite(out_path, img)
        print(f"\n🖼️ บันทึกรูปผลลัพธ์แล้ว!")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    run_dataset_test()
