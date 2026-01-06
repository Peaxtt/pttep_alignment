# 🛰️ PTTEP Alignment Package

แพ็กเกจ ROS 2 สำหรับคำนวณการทำ Alignment ระหว่างพิกัดจาก GPS และ LIDAR (SLAM) เพื่อหาค่า Transformation Matrix ($T$) ในรูปแบบ 2D (x, y, yaw) พร้อมระบบตรวจสอบความถูกต้องของข้อมูล (Data Validation)

## -Prerequisites (สิ่งที่ต้องติดตั้งก่อน)

เพื่อให้แพ็กเกจสามารถคอมไพล์และทำงานได้อย่างสมบูรณ์ จำเป็นต้องติดตั้ง Library พื้นฐานดังนี้:

### 1. ROS 2 Dependencies
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-geometry-msgs \
                 ros-$ROS_DISTRO-nav-msgs \
                 ros-$ROS_DISTRO-std-srvs \
                 ros-$ROS_DISTRO-rosidl-default-generators \
                 ros-$ROS_DISTRO-rosidl-default-runtime
```
### 2. Linear Algebra Library
```bash
sudo apt install libeigen-dev
```

## - Package Structure
โครงสร้างไฟล์ภายในแพ็กเกจประกอบด้วย:
```
pttep_alignment/
├── CMakeLists.txt          # การตั้งค่า Build และการเชื่อมโยง Eigen/Interfaces
├── package.xml             # รายการ Dependencies และข้อมูล Metadata
├── src/
│   └── alignment_node.cpp  # โค้ดหลัก: คำนวณ SVD Solver, Time Sync, Frame Tracking
└── srv/
    └── CalculateTransformation.srv # Custom Service Definition (bool reset)
```

## - Installation (ขั้นตอนการติดตั้ง)
1. รัน cmd นี้บน Terminal นี้หลังจากลงไฟล์และ Library ทั้งหมดเรียบร้อยแล้ว
```bash
# ใส่ตำแหน่งของ Workspace ให้ถูก
cd ~/ros2_ws 
colcon build --packages-select pttep_alignment
source install/setup.bash
```

2. รัน cmd นี้บน Terminal โดยเปลี่ยน pose_topic และ gps_topic ให้ตรงกัน Topic จริง
```bash
# หลังจาก build และ source แล้ว
ros2 run pttep_alignment alignment_node --ros-args \
  -r /current_pose:=/ชื่อ_topic_slam_จริง \
  -r /gps:=/ชื่อ_topic_gps_จริง
```

## - Usage Guide (วิธีการใช้งาน)
1. เริ่มรันโหนดหลัก (Solver Node)
```bash
ros2 run pttep_alignment alignment_node
```
2. บันทึกคู่จุดพิกัด (Save Location)เรียกใช้ Service เพื่อบันทึกพิกัดปัจจุบันจากทั้ง GPS และ SLAM (ต้องเก็บอย่างน้อย 3 จุด)
```bash
ros2 service call /save_location std_srvs/srv/Trigger
```
3. สั่งคำนวณหาผลลัพธ์ (Calculate)สั่งให้ระบบคำนวณหา Matrix $T$ โดยส่งค่า `reset: false`
```bash
ros2 service call /calculate_transformation pttep_alignment/srv/CalculateTransformation "{reset: false}"
```
4. ล้างข้อมูลใน Memory (Reset)หากต้องการลบจุดที่บันทึกไว้ทั้งหมดเพื่อเริ่มการทำ Alignment ใหม่ ให้ส่งค่า `reset: true`
```bash
ros2 service call /calculate_transformation pttep_alignment/srv/CalculateTransformation "{reset: true}"
```
