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
- **2D/3D simulation** with software-enforced joint angle limits and dynamic geometric rules

Total hardware cost: ~USD 350 vs USD 30,000–150,000 for equivalent industrial arms.

---

## 🔗 Project Links

| Resource | Link |
|----------|------|
| 🌐 Landing Page | [ferchng.github.io/brazo-robotico](https://ferchng.github.io/brazo-robotico) |
| 📄 Technical Report | [View PDF](https://github.com/ferchng/brazo-robotico/blob/main/docs/informe_brazo_robotico.pdf) |
| 📸 Photo Gallery | [Google Photos](https://photos.app.goo.gl/9NATdUXzkGk8ssc47) |
| 🎥 Demo Videos | *(coming soon)* |

---

## 🧱 Based On

This project builds upon the open-source [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) design by [TheRobotStudio](https://github.com/TheRobotStudio), published under the Apache 2.0 license.

> 📦 **Bill of Materials, CAD files, and hardware assembly instructions** are available in the original [SO-ARM101 repository](https://github.com/TheRobotStudio/SO-ARM101). Our technical report (linked above) includes the specific components and pricing used in this build.

Key differences from the original:
- Custom direct kinematics in JavaScript (`kinematics.js`) inside an Electron desktop app
- 2D/3D simulation with dynamic geometric safety rules per joint
- Stereo vision pipeline planned (USB webcams / smartphones)
- Fine-tuned YOLO model for object classification (upcoming)

---

## ⚙️ Hardware

| Component | Specs |
|-----------|-------|
| Servomotors | 6× Feetech STS3215 (12V, 30kg·cm) |
| Driver | Waveshare Bus Servo Adapter (A) V1.1 |
| Microcontroller | ESP32 |
| Vision (prototype) | 2× smartphones via USB (stereo) |
| Vision (production) | 2× USB webcams (fixed, stereo) |
| Frame | 3D printed (PLA MAX, Creality Ender 3 V3 SE) |
| Base | Recycled 40×40cm pressed wood board |
| Power | External 12V supply |

---

## 🧠 Software Stack

| Layer | Technology |
|-------|-----------|
| Desktop app | Electron (Node.js) |
| Direct kinematics | JavaScript (`kinematics.js`) |
| 3D render | Canvas / Three.js |
| Simulation (Python) | Python — geometric validation scripts |
| Simulation (web) | HTML + JavaScript prototypes |
| Object detection | YOLO (Ultralytics) with fine-tuning — upcoming |
| Vision pipeline | Python + OpenCV (stereo, upcoming) |
| Firmware | Arduino IDE (ESP32) |
| Dev tools | VS Code, OpenAI Codex, GitHub |

---

## 📁 Repository Structure

```
brazo-robotico/
├── app/                             # Desktop app (Electron) + WebSerial UI
│   ├── brazo_desktop_app/           # Main Electron app: 3D render, kinematics, serial control
│   └── ui_servos_webserial/         # Browser-based WebSerial interface
├── firmware/                        # Arduino/ESP32 sketches
│   ├── diagnostico_ping_sts3215_esp32/
│   ├── setear_ids_sts3215_arduino/
│   ├── leer_6_sts3215_bus_esp32/
│   ├── mover_un_motor_por_angulo_esp32/
│   └── mover_un_motor_por_angulo_esp32_test_reglas/  # ← main active sketch
├── kinematics/                      # Python geometric simulation and kinematics
│   └── simulacion_3d_simplificada/
├── simulation/                      # Web-based 3D simulator prototypes (HTML + JS)
├── vision/                          # Object detection pipeline (upcoming)
├── docs/                            # Technical report
│   └── informe_brazo_robotico.pdf
├── index.html                       # Landing page (GitHub Pages)
└── README.md
```

> ⚠️ **Pre-alpha:** code is actively being tested and debugged. `node_modules/` is excluded from the repo.

---

## 🚧 Development Status

| Module | Status |
|--------|--------|
| Mechanical assembly | ✅ 100% complete |
| Servo ID assignment & telemetry | ✅ Complete |
| Fixed + dynamic joint angle limits | ✅ Complete |
| 2D/3D geometric simulation | ✅ Functional prototype |
| Electron desktop app | ✅ Functional (active development) |
| Direct kinematics (PC → arm) | 🔄 ~80% — polishing XYZ coordinates |
| Stereo vision (smartphones → PC) | ⏳ Planned (Jul–Aug 2026) |
| YOLO fine-tuning & integration | ⏳ Planned (Jul–Aug 2026) |
| Full sorting demo | ⏳ Target: Nov 2026 |

---

## 🔩 Joint Angle Limits

Fixed limits per motor:

| Motor | Range |
|-------|-------|
| M1 | 60° – 300° |
| M2 | 90° – 270° |
| M3 | 90° – 270° |
| M4 | 75° – 260° (+ dynamic rules) |
| M5 | 0° – 340° |
| M6 | 0° – 140° (gripper, with offset) |

M4 also has dynamic geometric rules based on M2 and M3 pose to prevent collisions.

---

## 🚀 Getting Started

> Full setup instructions coming with alpha release.

### Prerequisites

- Node.js + npm (for Electron app)
- Python 3.10+ (for simulation scripts)
- Arduino IDE 2.x (for ESP32 firmware)
- Feetech STS3215 servos × 6
- Waveshare Bus Servo Adapter (A) V1.1
- External 12V power supply

### Quick Start *(pre-alpha)*

```bash
git clone https://github.com/ferchng/brazo-robotico.git
cd brazo-robotico

# Desktop app
cd app/brazo_desktop_app
npm install
npm start

# Python simulation
cd ../../kinematics/simulacion_3d_simplificada
python visualizador_brazo_con_reglas.py
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
