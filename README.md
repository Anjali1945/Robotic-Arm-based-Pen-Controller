# Robotic Arm Based Pen Controller

B.Tech Minor Project | ECE-327 | MANIT Bhopal | Group 35

---

## What it does

- Captures an image using a phone camera (IVCam over USB)
- Processes it using Python and OpenCV
- Converts it to G-code and sends it to an Arduino
- The robotic arm draws it on paper with a pen — fully automated

---

## Two Approaches

### Approach 1 — Multi-tool Workflow (Initial)

This was the first version. Each stage was a separate tool and required manual handoff.

- Image imported into **Inkscape** for manual vectorization
- SVG fed into **JSCut** to generate G-code
- G-code loaded into **UGS (Universal G-code Sender)** and sent to Arduino
- Any change meant restarting the whole chain from scratch
- Produced better results for complex shapes, but needed constant manual steps

### Approach 2 — Unified Python Pipeline (Final)

Replaced all four tools with a single Python script.

- IVCam streams live video directly into the Python environment
- Press S to snapshot — processing starts immediately
- OpenCV handles grayscale conversion, thresholding, and noise removal
- Scikit-image skeletonizes the binary image into single-pixel paths
- Python generates G-code from the paths and streams it over serial to Arduino
- Arduino (GRBL firmware) drives three stepper motors for X, Y, and Z axes

---

## Hardware

| Component | Qty | Role |
|-----------|-----|------|
| Arduino UNO (ATMega328P) | 1 | Main controller |
| Stepper Motor 28BYJ-48 (5V) | 3 | X / Y / Z axis motion |
| ULN2003 Driver Module | 3 | Motor current amplification |
| MB102 Power Supply Module | 1 | 5V / 12V regulated supply |

---

## Software

- Python, OpenCV, NumPy, Scikit-image, pySerial
- IVCam (mobile webcam app)
- Arduino IDE + GRBL firmware

---

## Setup

```bash
pip install opencv-python numpy scikit-image pyserial
python main.py
```

- Flash GRBL onto Arduino UNO
- Connect IVCam on phone and PC
- Wire each stepper motor through a ULN2003 driver to the Arduino

---

## Controls

| Key | Action |
|-----|--------|
| S | Snapshot and start drawing |
| P | Pause |
| R | Resume |
| Q / W / E | Jog +Z / +Y / -Z |
| A / D | Jog -X / +X |
| X | Jog -Y |

---

## Results

| Input | Accuracy | Output Quality |
|-------|----------|----------------|
| Straight lines | ~90% | Clean and stable |
| Basic shapes | ~70% | Minor distortion |
| Handwritten text | ~40% | Broken strokes |
| Complex images | ~30% | Heavy distortion |

---

## Limitations

- Skeletonization loses fine detail on complex inputs and small text
- No feedback loop — arm doesn't self-correct if it drifts
- Vibration and step loss affect output at higher speeds

---

## Future Scope

- Replace skeletonization with a learned model for better accuracy
- Support real-time video tracing
- Use conductive ink pen for basic PCB drawing on substrates

---

## Team

Anjali Thaware · Shreya Kumar · Neha Jhariya

Guide: Dr. Manish Kashyap
Department of ECE, MANIT Bhopal — April 2026
