# Offline Speaker-Aware Voice Control on ESP32

A fully offline, security-oriented voice control system implemented on ESP32 using TinyML.

This project was developed as part of a Bachelor’s degree thesis in Electrical Engineering.

---

## 🎯 Project Overview

This system combines:

- Keyword Spotting (KWS)
- Speaker Identification
- Security-first decision logic

Unlike cloud-based assistants, the entire inference pipeline runs locally on ESP32 without any internet dependency.

---

## 🔐 Motivation

Most smart home voice assistants rely on cloud processing and focus only on keyword detection.

This project explores a different approach:

- Fully offline operation
- Speaker-aware activation
- Prioritization of security over convenience
- Designed to prevent false activation by unauthorized users

---

## ⚙️ System Architecture

1. Audio acquisition via INMP441 (I2S microphone)
2. Feature extraction using Mel Filterbank Energies (MFE)
3. 1D Convolutional Neural Network (TinyML)
4. Margin-based decision scoring
5. Two-stage confirmation logic
6. Refractory timing to prevent double triggering

---

## 🧠 Machine Learning Pipeline

- Platform: Edge Impulse
- Feature extraction: Mel Filterbank Energies (MFE)
- Model type: Lightweight 1D CNN
- Quantization: INT8
- Inference mode: Continuous
- Deployment target: ESP32 DevKit

Official reference:
https://docs.edgeimpulse.com/tutorials/end-to-end/sound-recognition

---

## 📊 Model Performance

- Overall accuracy: ~96%
- Low false activation rate for unauthorized users
- Balanced train/test split: 80% / 20%
- Robust separation between classes

See `/results` for detailed evaluation plots.

---

## 🎛 System Behavior

| Spoken Command        | System Response |
|----------------------|----------------|
| switch_on_me         | Green LED ON (latched) |
| Power Off            | Green LED OFF |
| switch_on_other      | Red LED warning (2s) |
| unknown / noise      | No action |

Security logic prefers false rejection over false acceptance.

---

## 🛠 Hardware Setup

### Components
- ESP32 DevKit (ESP-WROOM-32)
- INMP441 I2S Microphone
- Green LED
- Red LED
- Resistors

See `/hardware` for wiring diagram and pin mapping.

---

## 🔌 Pin Mapping

| ESP32 Pin | Function |
|------------|----------|
| GPIO14 | I2S SCK |
| GPIO25 | I2S WS |
| GPIO32 | I2S SD |
| GPIO2  | Green LED |
| GPIO4  | Red LED |

---

## 🚀 Installation & Deployment

1. Install Arduino IDE
2. Install ESP32 board support
3. Import Edge Impulse library ZIP:
   - Sketch → Include Library → Add .ZIP Library
4. Open `firmware/esp32_voice_control.ino`
5. Select ESP32 Dev Module
6. Upload to board
7. Open Serial Monitor (115200 baud)

---

## 📂 Repository Structure

- `/firmware` → Arduino source code
- `/ei-model` → Exported Edge Impulse library
- `/hardware` → Wiring diagrams and pinout
- `/results` → Accuracy and evaluation plots
- `/docs` → Thesis and technical documentation

---

## ⚠ Limitations

- Performance decreases in noisy environments
- Limited dataset diversity
- Sensitivity trade-off due to security prioritization

---

## 🔮 Future Work

- Larger dataset
- Multi-user personalization
- Custom PCB design
- Noise robustness improvements
- Academic publication

---

## 📜 License

MIT License

---

## 👨‍🎓 Academic Context

Developed as a Bachelor’s thesis project in Embedded Systems and TinyML.
