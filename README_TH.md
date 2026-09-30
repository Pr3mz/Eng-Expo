# ExpoRedCrObot (Field-Ready Edition 🏆)

โปรเจกต์นี้คือเวอร์ชันสมบูรณ์ที่สุดสำหรับหุ่นยนต์คัดแยกพลอย 6 สี (ExpoRedCrObot) โดยถูกออกแบบมาให้ **"พร้อมแข่ง (Field-Ready)"** และทนทานต่อสภาพแวดล้อมหน้างานจริง (แสงเปลี่ยน, เงาบัง, อินเทอร์เน็ตหลุด)

ไฟล์หลักที่ใช้รันโปรแกรมคือ: **`Field_Ready_Project/Final1.py`**

---

## 🗺️ System Flowchart (สถาปัตยกรรมระบบ)

```mermaid
flowchart TD
    Start([เปิดกล้องและเชื่อมต่อหุ่นยนต์ผ่าน UDP]) --> Check{รอคำสั่งจากผู้ใช้}

    Check -->|กด M| Manual[Manual Mode]
    Manual --> G[ควบคุมด้วยท่าทางมือ MediaPipe]
    
    Check -->|กด Spacebar| Auto[Auto Mode (Autonomous)]
    Auto --> Vision[ตรวจจับพิกัดด้วย AI Roboflow]
    Vision --"พิกัดเพชร และ หุ่นยนต์"--> Planner{State Machine}
    
    Planner -->|1| SeekGem[SEEKING_GEM: วิ่งเข้าหาเพชร]
    SeekGem -->|2| Grab[GRABBING_GEM: ก้มเก็บเพชร]
    Grab -->|3| SeekDrop[SEEKING_DROP: วิ่งไปจุดสีเดียวกัน]
    SeekDrop -->|4| Drop[DROPPING_PAYLOAD: ปล่อยของ & ถอยหลัง]
    Drop -->|วนลูปกลับ| SeekGem
```

---

## ✨ ฟีเจอร์เด่นของเวอร์ชันนี้ (Hybrid AI + CV)

1. **AI Object Detection (Roboflow):** 
   - ใช้ AI กรองสิ่งรบกวน พร้อมวาด UI (Bounding Box) สวยงาม
2. **ระบบตาสำรองฉุกเฉิน (CV Fallback):**
   - หากอินเทอร์เน็ตหน้างานหลุด หุ่นจะสลับไปใช้ระบบแยกสี HSV ทันที
3. **ควบคุมด้วยท่าทางมือ (MediaPipe Gestures):**
   - สามารถควบคุมหุ่นยนต์ในโหมด Manual (M) ได้ด้วยการทำมือ (ชู 5 นิ้ว, ชู 4 นิ้วเลี้ยวซ้ายขวา, ชูนิ้วโป้งกับชี้เป็นรูปตัว L เพื่อสั่งหนีบเพชร)
4. **วงกลมบาเรียลบจุดบอด (Robot Exclusion Zone):**
   - รัศมีปกป้องตัวหุ่น 130 พิกเซล ป้องกัน AI ตรวจจับตัวเอง
5. **แก้ปัญหาสีเพี้ยน (Smart Twin Resolver):**
   - ชดเชยสี Cyan/Blue อัตโนมัติหากแสงเพี้ยน
6. **รองรับกล้องมือถือ (IP Webcam):**
   - ใช้มือถือ Android เป็นกล้องมุมสูงได้ผ่าน Wi-Fi

---

## 🚀 วิธีการใช้งาน (How to Run)

เปิด Terminal ใน VSCode แล้วพิมพ์คำสั่ง:
```bash
cd Field_Ready_Project
python3 Final1.py
```

### ขั้นตอนการ Setup ก่อนเริ่ม
1. **STEP 1:** คลิกเลือกมุมสนามทั้ง 4 มุม เพื่อให้ดัดภาพ Top-down
2. **STEP 2:** คลิกเลือกใจกลางจุดปล่อย (Drop Zones) ทั้ง 6 สีให้ครบ
3. **READY:** กดปุ่ม `Spacebar` เพื่อให้หุ่นคัดแยกเอง หรือกด `M` เพื่อบังคับเอง!

---

## ⌨️ คีย์ลัดควบคุม (Hotkeys)

- `Spacebar` : เริ่ม / หยุด โหมด Auto
- `[` และ `]` : ลด / เพิ่ม ความสว่างของกล้อง
- `Q` : ปิดโปรแกรม
- `R` : รีเซ็ตมุมสนามใหม่
- `M` : สลับโหมดบังคับ (Manual Mode) -> **โหมดนี้ใช้การชูมือทำท่าทางเพื่อบังคับหุ่นและกริปเปอร์**
