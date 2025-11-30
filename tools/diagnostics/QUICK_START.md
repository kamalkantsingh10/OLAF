# ODrive Quick Start - 2 Simple Scripts

## 📋 Your Scripts (Only These 2!)

```
tools/diagnostics/
├── odrive_test_and_calibrate.py  ← Run this FIRST (calibration)
└── odrive_demo.py                ← Run this SECOND (demo)
```

---

## 🚀 Step-by-Step

### 1️⃣ First Time: Calibrate

```bash
python tools/diagnostics/odrive_test_and_calibrate.py
```

✅ This does EVERYTHING:
- Finds ODrive
- Tests hall sensors
- Tests motor wires
- Calibrates motors
- Tests velocity control
- Saves config

**Time:** ~60 seconds

---

### 2️⃣ After Calibration: Demo

```bash
python tools/diagnostics/odrive_demo.py
```

🎮 Shows 5 cool demos:
1. Forward/reverse
2. Turning
3. Spin in place
4. Speed ramping
5. Individual motors

**Time:** ~30 seconds

---

## ✅ You're Done!

That's it! Just 2 scripts.

**Next:** Story 6.4 - MPU6050 IMU

---

## 💡 When to Use Each

| Script | When to Run |
|--------|-------------|
| `odrive_test_and_calibrate.py` | First time, or after wiring changes |
| `odrive_demo.py` | Anytime you want to see motors work |

---

## 🔧 Troubleshooting

**"ODrive not found"**
- Plug in USB
- Check green LED is on
- Try: `sudo usermod -a -G dialout $USER` then logout/login

**"Motors not calibrated"**
- Run calibration first!
- `python tools/diagnostics/odrive_test_and_calibrate.py`
