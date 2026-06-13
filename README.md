# 🦾 Robotic Arm with AI-Based Object Detection & Autonomous Sorting

**6-DOF robotic arm with computer vision for autonomous object detection and sorting.**  
Built at Instituto Industrial Luis A. Huergo, Buenos Aires · 2026

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-pre--alpha-orange.svg)]()
[![Based on](https://img.shields.io/badge/based%20on-SO--ARM101-lightgrey.svg)](https://github.com/TheRobotStudio/SO-ARM101)

---

## 📌 Overview

This project implements a low-cost 6 degree-of-freedom robotic arm capable of:

- **Autonomous object detection** using a fine-tuned YOLO model
- **Direct kinematics** for precise PC-to-arm coordinate translation
- **Autonomous sorting** based on object classification
- **2D/3D simulation** for software-enforced joint angle limits

Total hardware cost: ~USD 350 vs USD 30,000–150,000 for equivalent industrial arms.

---

## 🔗 Project Links

| Resource | Link |
|----------|------|
| 🌐 Landing Page | [ferchng.github.io/brazo-robotico](https://ferchng.github.io/brazo-robotico) |
| 📄 Technical Report | [View PDF](https://github.com/ferchng/brazo-robotico/blob/main/informe_brazo_robotico.pdf) |
| 📸 Photo Gallery | [Google Photos](https://photos.app.goo.gl/9NATdUXzkGk8ssc47) |
| 🎥 Demo Videos | *(coming soon)* |

---

## 🧱 Based On

This project builds upon the open-source [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) design by [TheRobotStudio](https://github.com/TheRobotStudio), published under the Apache 2.0 license.

> 📦 **Bill of Materials, CAD files, and hardware assembly instructions** are available in the original [SO-ARM101 repository](https://github.com/TheRobotStudio/SO-ARM101). Our technical report (linked above) includes the specific components and pricing used in this build.

Key differences from the original:
- Custom direct kinematics algorithm (Python, PC-side)
- Fine-tuned YOLO model for object classification
- 2D/3D simulation for hardware-safe angle limits
- Fixed camera architecture (under evaluation vs. wrist-mounted)

---

## ⚙️ Hardware

| Component | Specs |
|-----------|-------|
| Servomotors | 6× Feetech STS3215 (12V, 30kg·cm) |
| Driver | Waveshare Bus Servo Adapter (A) |
| Microcontroller | ESP32 (via Arduino IDE) |
| Camera | USB 1080p module (architecture TBD: fixed vs. wrist) |
| Frame | 3D printed (PLA MAX, Creality Ender 3 V3 SE) |
| Base | Recycled 40×40cm pressed wood board |

---

## 🧠 Software Stack

| Layer | Technology |
|-------|-----------|
| Object detection | YOLO (Ultralytics) with fine-tuning |
| Vision pipeline | MediaPipe + custom post-processing |
| Kinematics | Python (direct kinematics, custom implementation) |
| Simulation | Python 2D/3D (software joint angle limits) |
| Microcontroller | Arduino IDE (ESP32) |
| Dev tools | VS Code, OpenAI Codex, GitHub |

---

## 📁 Repository Structure

```
brazo-robotico/
├── kinematics/          # Direct kinematics algorithm (PC → arm)
├── simulation/          # 2D/3D simulation for angle limit validation
├── vision/              # YOLO pipeline and camera feed processing
├── firmware/            # Arduino/ESP32 firmware
├── docs/                # Spanish documentation
│   └── informe_brazo_robotico.pdf
├── index.html           # Landing page
└── README.md
```

> ⚠️ **Pre-alpha:** code is actively being tested and debugged. Structure may change.

---

## 🚧 Development Status

| Module | Status |
|--------|--------|
| Mechanical assembly | ✅ 100% complete |
| 2D/3D simulation | ✅ Functional prototype |
| Direct kinematics (PC → arm) | 🔄 ~80% — polishing base coordinates |
| Camera feed → kinematics commands | ⏳ Planned (Jul–Aug 2026) |
| YOLO fine-tuning & integration | ⏳ Planned (Jul–Aug 2026) |
| Full sorting demo | ⏳ Target: Nov 2026 |

---

## 🚀 Getting Started

> Full setup instructions coming with alpha release.

### Prerequisites

- Python 3.10+
- Arduino IDE 2.x
- Feetech STS3215 servos × 6
- Waveshare Bus Servo Adapter (A)

### Quick Start *(pre-alpha)*

```bash
git clone https://github.com/ferchng/brazo-robotico.git
cd brazo-robotico

# Install Python dependencies
pip install -r requirements.txt  # coming soon

# Run simulation
python simulation/sim_3d.py
```

---

## 👥 Team

| Name | Role |
|------|------|
| Stefano Cavallaro | Development |
| Fernando Chang | Development |
| Federico Ottolini | Development |

**Tutors:** Gonzalo Rodriguez Goris · Fabián Vega  
**Institution:** Instituto Industrial Luis A. Huergo, Buenos Aires

---

## 📄 License

This project is licensed under the MIT License.  
Hardware design credits: [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) by TheRobotStudio (Apache 2.0).
