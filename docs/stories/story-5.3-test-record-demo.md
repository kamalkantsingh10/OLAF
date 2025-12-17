# Story 5.3: Test and Record End-to-End Demo

**Epic:** Epic 5 - End-to-End Demo
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 3-4 hours

---

## User Story

**As a** builder,
**I want** to thoroughly test the demo and record a high-quality video,
**so that** I can showcase OLAF Phase 1 completion to the community.

---

## Acceptance Criteria

1. ✅ Demo runs successfully 5 times without failures
2. ✅ All 23 demo actions execute correctly
3. ✅ Video recorded (1080p minimum, 2-3 minutes)
4. ✅ Video includes multiple camera angles
5. ✅ Audio narration added (or subtitles)
6. ✅ Video edited and rendered to final format
7. ✅ Demo video uploaded to YouTube/repository

---

## Implementation Steps

### 1. Pre-Demo Checklist

```bash
# Hardware checks:
✓ Battery fully charged (42V)
✓ All mounting bolts tight
✓ Cables secured, no strain
✓ Emergency stop accessible
✓ Floor clean, flat, non-slippery
✓ Adequate space for movement (2m × 2m minimum)

# Software checks:
✓ All ROS2 drivers tested individually
✓ I2C devices responding (i2cdetect -y 1)
✓ Launch file brings up all nodes
✓ Demo script syntax checked
```

### 2. Perform Dry Runs

```bash
# Run 1: Check timing
#   - Note any awkward pauses
#   - Adjust sleep() durations in demo script

# Run 2: Check all actions execute
#   - Eyes blink ✓
#   - Ears move ✓
#   - Heart animates ✓
#   - Printer prints ✓
#   - Kickstand deploys/retracts ✓
#   - Robot balances ✓
#   - Robot moves forward/backward ✓

# Run 3: Test with narration
#   - Practice speaking over demo
#   - Time narration to match actions

# Run 4-5: Full rehearsals
#   - Run as if recording
#   - Check for any issues
```

### 3. Set Up Recording Equipment

**Camera setup:**

```bash
# Camera 1: Front angle (eye level)
#   - Shows eyes, ears, heart display
#   - Captures facial expressions

# Camera 2: Side angle (shows full robot)
#   - Shows balance, movement
#   - Captures kickstand deployment

# Camera 3: Close-up (optional)
#   - Shows printer output
#   - Shows heart animation detail

# Lighting:
#   - Bright, even lighting
#   - No harsh shadows
#   - White balance corrected

# Stabilization:
#   - Use tripods (no shaky handheld)
#   - Or phone gimbals
```

### 4. Record Demo (Multiple Takes)

```bash
# Take 1: Full run
#   - Record from all camera angles
#   - Note any issues

# Take 2: Backup (if Take 1 has issues)

# Take 3: B-roll footage
#   - Close-ups of components
#   - Individual module demos
#   - Behind-the-scenes assembly shots
```

### 5. Record Narration

**Script (from Story 5.1):**

```
# Option A: Live narration during recording
#   - Speak clearly into microphone
#   - Match timing to robot actions

# Option B: Voiceover in post-production
#   - Record audio separately
#   - Sync in video editor

# Option C: Subtitles only
#   - No voiceover
#   - Add text overlays in editor
```

### 6. Edit Video

**Editing steps:**

```bash
# Software: DaVinci Resolve (free) or iMovie
# 1. Import all video clips
# 2. Select best takes for each section
# 3. Cut and arrange on timeline
# 4. Add transitions (simple cuts, no fancy effects)
# 5. Color correction (consistent lighting)
# 6. Add audio:
#    - Narration track
#    - Optional background music (low volume, non-intrusive)
# 7. Add text overlays:
#    - Title: "OLAF - Open-source Lovable Assistive Friend"
#    - Section labels: "Head+Ears Module", "Torso Module", etc.
#    - Specs overlay: "36V Battery, 200Hz Balancing PID, ROS2 Humble"
# 8. Add credits:
#    - "Built by [Your Name]"
#    - "GitHub: github.com/[repo]/OLAF"
# 9. Export: 1080p MP4, H.264 codec, 30fps
```

### 7. Create YouTube Upload

**Video details:**

```
Title: OLAF Phase 1 Demo - Self-Balancing Robot with ROS2

Description:
OLAF (Open-source Lovable Assistive Friend) is a self-balancing robot built with ROS2, ESP32 microcontrollers, and a Raspberry Pi 5 with Hailo AI acceleration.

Phase 1 Features:
- Self-balancing on two wheels (200Hz PID control)
- Expressive OLED eyes and articulated ears
- Heart-shaped display with animations
- Thermal printer for messages
- Motorized kickstand and head tilt
- 36V hoverboard battery and motors

Hardware:
- Raspberry Pi 5 + Hailo-8L AI Kit
- 4× ESP32 microcontrollers (I2C slaves)
- ODrive S1 motor controller
- MPU6050 IMU
- 2× Hoverboard BLDC motors
- Skateboard trucks for suspension

Software:
- ROS2 Humble
- Python driver nodes
- Arduino firmware (PlatformIO)
- 200Hz balancing control loop

Build your own: [GitHub link]
Documentation: [GitHub docs link]

Tags: robotics, ros2, self-balancing, esp32, raspberry-pi, open-source, arduino, 3d-printing

Thumbnail: Eye-catching shot of robot balancing with LEDs lit
```

### 8. Upload to GitHub

```bash
# Create docs/videos/ directory
mkdir -p ~/olaf/docs/videos

# Add README with video links
echo "# OLAF Demo Videos

## Phase 1 Demo
- YouTube: https://youtu.be/[video-id]
- Duration: 2:45
- Date: 2025-12-17

### Showcased Features:
- Self-balancing (200Hz PID)
- Head+Ears expressions
- Heart display animations
- Thermal printer output
- Motorized kickstand
- Forward/backward movement

### Behind the Scenes:
[Links to assembly timelapses, component tests, etc.]
" > ~/olaf/docs/videos/README.md

# Commit and push
cd ~/olaf
git add docs/videos/
git commit -m "docs: add Phase 1 demo video"
git push
```

### 9. Share on Social Media

```bash
# Platforms:
#   - Reddit: r/robotics, r/ROS, r/arduino, r/raspberry_pi
#   - Hackaday.io: Create project page
#   - Hackster.io: Submit tutorial
#   - Twitter/X: #robotics #ROS2 #OpenSource
#   - LinkedIn: Professional network

# Post template:
"Just completed Phase 1 of OLAF - a self-balancing robot with ROS2!

Features: 200Hz PID balancing, expressive eyes/ears, thermal printer, and full ROS2 integration.

All hardware/software is open-source. Check out the demo and build docs:
[YouTube link]
[GitHub link]

#robotics #ROS2 #OpenSource #DIY"
```

### 10. Document Lessons Learned

**Create reflection document:**

```markdown
# Phase 1 Demo: Lessons Learned

## What Went Well:
- All systems integrated successfully
- Balancing PID tuned to stable performance
- ROS2 architecture enabled clean module separation
- Demo script choreographed smoothly

## Challenges Overcome:
- I2C bus conflicts (resolved with proper pull-ups)
- ODrive calibration complexity (documented thoroughly)
- PID tuning iterative process (final gains documented)
- Power distribution design (2oz copper essential)

## Technical Highlights:
- 200Hz control loop achieved on ESP32
- Kitchen bin enclosure surprisingly effective
- Skateboard trucks provide excellent suspension
- Hoverboard motors perfect for this application

## Future Improvements:
- Add encoder odometry for better localization
- Implement obstacle avoidance with OAK-D camera
- Add ROS2 Nav2 for autonomous navigation
- Improve cable management (custom cable chains)

## Community Feedback:
[Add after sharing publicly]

## Statistics:
- Total build time: ~6 months (part-time)
- Total cost: ~$800
- Lines of code: ~5,000
- 3D printed parts: 15
- Custom PCBs: 4
- GitHub stars: [TBD]
```

---

## Testing & Validation

**Test 1: Demo Success Rate**
```bash
# 5/5 successful runs (100%)
```

**Test 2: Video Quality**
```bash
# 1080p resolution, clear audio, smooth editing
```

**Test 3: Community Reception**
```bash
# Positive comments, interest in build guide
```

---

## Dependencies

**Before this story:**
- Story 5.2: Create Demo Launch File ✅

**After this story:**
- Story 5.4: Document Phase 1 Completion

---

## References

- [YouTube Video Best Practices](https://creatoracademy.youtube.com/)
- [DaVinci Resolve Tutorial](https://www.blackmagicdesign.com/products/davinciresolve/training)

---

## Notes

- **Multiple Takes:** Plan for 2-3 full demo runs (expect 1-2 to have minor issues)
- **Battery Management:** Fully charge between takes
- **Backup Plan:** Have E-stop ready, test area clear
- **Editing Time:** Allow 2-4 hours for video editing
- **YouTube Upload:** May take 30-60 minutes for processing

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
