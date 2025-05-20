# 🤖 Smart Maze Solving Robot

This project presents the design and implementation of a **smart maze-solving robot** that leverages the **Rapidly-Exploring Random Tree (RRT)** algorithm to efficiently navigate complex mazes. Built using a **Raspberry Pi**, the robot identifies the shortest path in real-time, collects waypoints, and follows them with high precision. It serves as a showcase of advanced pathfinding in robotics using both RRT and A* algorithms.

---

## 🧠 Key Features

- 🔍 **Maze Pathfinding** using RRT and A* algorithms  
- 🗺️ **Waypoint Collection & Navigation**  
- 📷 **Maze Image Acquisition** through camera + image processing  
- ⚙️ **Raspberry Pi-controlled robot** with motor driver & sensors  
- 💻 **Remote monitoring** using RealVNC & Python control  
- 🧪 **Tested on multiple maze configurations**  

---

## 🛠️ Tech Stack

- **Hardware**
  - Raspberry Pi (Quad-core Cortex-A72, Wi-Fi, BLE)
  - DC Motors, Motor Driver, Chassis
  - Ultrasonic Sensors, Line Sensors
  - Camera Module (top-down maze capture)

- **Software**
  - Python 3
  - OpenCV (for image processing)
  - PyCharm (IDE)
  - RealVNC (for remote GUI access)
  - Raspbian OS (via Raspberry Pi Imager)

---

## 📁 Project Structure

├── Code/ # Python code for RRT, A*
├── Images/ # Captured maze images and processed outputs
├── Hardware/ # Wiring diagrams and chassis images
├── Results/ # Screenshots and performance logs
├── README.md # Project documentation

---

## 🚀 Getting Started

1. **Setup Raspberry Pi**:
   - Install Raspbian OS using Raspberry Pi Imager
   - Enable SSH and VNC for remote access

3. **Install Dependencies**:
   ```bash
   sudo apt update
   sudo apt install python3 python3-pip
   pip3 install numpy opencv-python
   
3.Run the Code:

Capture top-view image of the maze

Process image and run RRT/A* script to generate waypoints

Robot follows path via Raspberry Pi motor control

📊 Results

✅ Successfully navigated multiple maze configurations

⏱️ Improved solving time using optimized path planning

🧠 Real-time path re-planning and precision movement

🛡️ Consistent and robust performance across dynamic environments


👥 Contributors

Rushikesh K

Maanasa Pamarthy

Mansi Talla
