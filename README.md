# 🤖 5DOF Bioprinter & Robotics Control – MATLAB & Arduino Integration

## 📌 Introduction
This project contains a collection of MATLAB scripts and Arduino sketches developed for simulating, controlling, and testing a **5‑Degree‑of‑Freedom (DOF) robotic bioprinter** and related mechatronic systems.  
It demonstrates expertise in **robot kinematics, computer vision, dynamic simulation, and microcontroller‑based hardware control** — from algorithm design to real‑time implementation.

---

## ⚙️ Project Components

### 🔹 MATLAB Scripts
- **DirectKinematicsBasic.m** – Calculates forward kinematics for a 5DOF manipulator.  
- **TriangulatedImages.m** – Processes static images for 3D position estimation via triangulation.  
- **TriangulatedLiveCamera.m** – Captures and processes live camera feeds for real‑time 3D localization.  
- **doublePendulumBasic.m** – Simulates the dynamics of a double pendulum system.  
- **doublePendulumLiveCamera.m** – Tracks and analyzes a physical pendulum setup via camera input.  
- **simulation.m / simulationpart2.m** – Full system simulations integrating mechanical models and control logic.

### 🔹 Arduino Sketches
- **bingo_doublependulum.ino** – Drives sensors/actuators for a double pendulum test platform.  
- **bingo_pendulum.ino** – Single pendulum setup with real‑time data acquisition.  
- **verifyPCA9685.ino** – Tests PCA9685 PWM driver functionality for servo control.

### 🔹 Other Files
- **CreatingNetworkSocket** – Network communication setup for remote control or data streaming between systems.

---

## 📊 Key Deliverables
- Forward kinematics implementation for custom 5DOF robotic arm.  
- Real‑time camera triangulation for object position tracking.  
- Simulations of dynamic systems (pendulums, robotic motion).  
- Verified microcontroller firmware for actuator control and sensor integration.  
- Modular code adaptable to other robotics platforms.

---

## ✅ Conclusion
This repository showcases:
- Integration of MATLAB‑based simulation and vision processing with Arduino hardware control.  
- Application of robotics theory (kinematics & dynamics) to functional prototypes.  
- Experience bridging simulation, embedded systems, and real‑time data acquisition.  

---

## 💻 Tech Stack
- **MATLAB** (Simulations, computer vision, kinematics)  
- **Arduino C/C++** (Microcontroller programming)  
- **PCA9685 PWM Driver** (Servo control)  
- **Camera systems** for real‑time object tracking

---

## 📈 Next Steps
- Implement inverse kinematics for precise end‑effector positioning.  
- Integrate closed‑loop control with sensor feedback.  
- Expand live vision tracking to multi‑object scenarios.  
- Prepare documentation for full bioprinting pipeline integration.

---

## 🤝 Contact
Created by **[Your Name]**  
🔗 [LinkedIn](#) | 🌐 [Portfolio](#)
