# Raspberry Pi 5 Setup Guide for OLAF

Complete setup guide for the Raspberry Pi 5 as OLAF's orchestration layer. This guide covers:

- **Part 1-2**: Ubuntu 24.04 and ROS2 Jazzy installation
- **Part 3**: ROS2 workspace setup
- **Part 4**: I2C configuration for ESP32 modules
- **Part 5**: USB serial for Waveshare servo adapters
- **Part 6**: Basic Fusion HAT installation
- **Part 7**: Hailo-8L AI Kit setup (26 TOPS for local Whisper STT)
- **Part 8**: Detailed Fusion HAT configuration (WS2812, PWM, I2S)
- **Part 9**: Multi-machine ROS2 networking
- **Part 10**: Git configuration
- **Part 11**: Display rotation (optional)

**Related Stories:**
- Story 0.1: Install Ubuntu 24.04 and ROS2 Jazzy
- Story 0.3: Configure Multi-Machine ROS2 Networking
- Story 0.5: Configure Fusion HAT and I2C
- Story 0.6: Configure USB Serial for Waveshare Adapters

---

## Hardware Overview

The Raspberry Pi 5 serves as OLAF's brain, running ROS2 nodes that coordinate all robot subsystems. It communicates with hardware through multiple interfaces, with two HATs stacked on top providing AI acceleration and GPIO expansion.

```
┌─────────────────────────────────────────────────────────────┐
│              HARDWARE STACK (TOP TO BOTTOM)                 │
├─────────────────────────────────────────────────────────────┤
│  ┌───────────────────────────────────────────────────────┐  │
│  │   Hailo-8L AI HAT (26 TOPS)                           │  │
│  │   └─ Local Whisper STT (<200ms latency)               │  │
│  └───────────────────────────────────────────────────────┘  │
│                          ↓ PCIe                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │   Sunfounder Fusion HAT+                              │  │
│  │   ├─ WS2812 LEDs (24 indicator lights)                │  │
│  │   ├─ PWM (kickstand servos)                           │  │
│  │   └─ I2S Audio (speaker)                              │  │
│  └───────────────────────────────────────────────────────┘  │
│                          ↓ GPIO                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │   Raspberry Pi 5 (16GB)                               │  │
│  │   ├─ Ubuntu 24.04 LTS + ROS2 Jazzy                    │  │
│  │   ├─ I2C Bus → Head ESP32 (0x10), Base ESP32 (0x11)   │  │
│  │   └─ USB Serial → Waveshare Neck, Waveshare Ears      │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

**Why this architecture?**
- **Hailo-8L**: Provides 26 TOPS of AI acceleration for local speech-to-text. No cloud latency, no privacy concerns, works offline.
- **Fusion HAT**: Pre-built GPIO expansion providing WS2812 LED control and PWM outputs without custom PCB work or soldering.
- **I2C**: Low-latency communication with ESP32s running real-time control loops (eye animations at 60fps, balancing PID at 200Hz)
- **USB Serial**: Reliable communication with Waveshare servo adapters — no soldering required, plug-and-play

---

## Part 1: Ubuntu 24.04 LTS Installation

We use Ubuntu 24.04 LTS instead of Raspberry Pi OS because ROS2 Jazzy (the latest long-term support release) has native packages for Ubuntu 24.04. This means easier installation and better compatibility with the ROS2 ecosystem.

### 1.1 Download and Flash

First, we need to create a bootable SD card with Ubuntu. The Raspberry Pi Imager tool makes this easy and lets us pre-configure WiFi and SSH so the Pi is accessible immediately on first boot.

1. **Download Ubuntu Server 24.04 LTS (64-bit)** for Raspberry Pi from:
   - https://ubuntu.com/download/raspberry-pi

2. **Install and run Raspberry Pi Imager** on your PC:
   ```bash
   sudo apt install rpi-imager
   rpi-imager
   ```

3. **Configure the image** in Raspberry Pi Imager:
   - Choose OS: Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)
   - Choose Storage: Select your SD card
   - Click the gear icon to configure:
     - Set hostname: `olaf`
     - Enable SSH with password authentication
     - Set username and password
     - Configure WiFi (SSID and password)
     - Set locale settings

4. **Write the image** and wait for verification to complete

5. **Insert the SD card** into your Pi 5 and power it on

The Pi will boot and automatically connect to your WiFi network. Give it about 2 minutes for the first boot to complete.

### 1.2 Initial Configuration

Now we'll SSH into the Pi and perform initial system configuration. These steps ensure the system is up-to-date and properly configured for development.

**Connect to the Pi via SSH:**
```bash
ssh kamal@olaf.local
```

If `olaf.local` doesn't resolve, find the Pi's IP address from your router and use that instead.

**Update the system packages:**

This downloads the latest package lists and upgrades all installed packages to their newest versions. Important for security and compatibility.

```bash
sudo apt update && sudo apt upgrade -y
```

**Fix locale settings:**

Ubuntu Server sometimes has incomplete locale configuration, which causes warning messages. These commands generate the proper locale files and set them as default.

```bash
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
echo 'export LANG=en_US.UTF-8' >> ~/.bashrc
source ~/.bashrc
```

**Set your timezone:**

Replace the timezone with your own (use `timedatectl list-timezones` to see options).

```bash
sudo timedatectl set-timezone America/Los_Angeles
```

---

## Part 2: ROS2 Jazzy Installation

ROS2 (Robot Operating System 2) is the middleware framework that enables all OLAF's subsystems to communicate. We're using Jazzy Jalisco, the latest LTS release supported until 2029.

ROS2 provides:
- **Pub/Sub messaging**: Nodes publish and subscribe to topics (e.g., `/head/expression`)
- **Services**: Request/response patterns for commands
- **Actions**: Long-running tasks with feedback
- **Launch system**: Start multiple nodes with one command
- **Ecosystem**: Navigation, visualization, and debugging tools

### 2.1 Add ROS2 Repository

ROS2 isn't in Ubuntu's default repositories, so we need to add the official ROS2 package repository. This involves adding a GPG key (to verify package authenticity) and the repository URL.

**Install prerequisite tools:**
```bash
sudo apt install -y software-properties-common curl
```

**Add the ROS2 GPG key:**

This key verifies that packages actually come from the ROS2 project and haven't been tampered with.

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

**Add the ROS2 repository:**

This command constructs the correct repository URL for your Ubuntu version and adds it to apt's sources.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.2 Install ROS2 Jazzy

Now we can install ROS2. We're installing `ros-jazzy-ros-base` (not `desktop`) because the Pi doesn't need GUI tools — those run on your development PC.

**Update apt and install ROS2:**
```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-base
```

**Install development tools:**

- `colcon`: The build tool for ROS2 workspaces
- `rosdep`: Automatically installs dependencies for ROS2 packages
- `pip`: Python package manager for additional libraries

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip
```

**Initialize rosdep:**

rosdep maintains a database of system dependencies for ROS2 packages. We need to initialize it once.

```bash
sudo rosdep init
rosdep update
```

### 2.3 Configure ROS2 Environment

Every terminal session needs to "source" ROS2's setup script to access ROS2 commands. We'll add this to `.bashrc` so it happens automatically.

We also set two important environment variables:
- `ROS_DOMAIN_ID=42`: Isolates our robot's traffic from other ROS2 systems on the network. All machines communicating must use the same ID.
- `ROS_LOCALHOST_ONLY=0`: Allows ROS2 to communicate across the network (not just localhost).

```bash
echo "" >> ~/.bashrc
echo "# ROS2 Jazzy" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

source ~/.bashrc
```

### 2.4 Verify ROS2 Installation

Let's confirm ROS2 is working properly:

```bash
echo $ROS_DISTRO
```
Expected output: `jazzy`

```bash
ros2 topic list
```
Expected output: `/parameter_events` and `/rosout` (default topics that always exist)

```bash
ros2 doctor --report
```
This shows detailed system information about your ROS2 installation.

---

## Part 3: ROS2 Workspace Setup

Since this Pi is dedicated to OLAF, we'll use a simple structure: clone the repository directly to `~/olaf/` and use the `ros2/` subdirectory as the colcon workspace.

### 3.1 Clone Repository

Clone the OLAF repository directly to your home directory:

```bash
cd ~
git clone https://github.com/kamalkantsingh10/OLAF.git olaf
```

Note: We use HTTPS for simplicity. If you prefer SSH (for passwordless push), set up SSH keys per Part 10, then use: `git clone git@github.com:kamalkantsingh10/OLAF.git olaf`

**Resulting structure:**
```
~/olaf/                    # Git repo (home base for everything)
├── ros2/
│   ├── src/               # ROS2 packages (olaf_head, olaf_base, etc.)
│   ├── build/             # colcon output (gitignored)
│   ├── install/           # colcon output (gitignored)
│   └── log/               # colcon output (gitignored)
├── modules/               # ESP32 firmware (head, base)
├── docs/                  # Documentation
└── tools/                 # Diagnostic scripts
```

### 3.2 Install Python Dependencies with Poetry

OLAF uses Poetry for Python dependency management. The root `pyproject.toml` defines all required packages including:
- `smbus2`: I2C communication with ESP32 modules
- `pyserial`: USB serial communication with Waveshare adapters
- `numpy`: Numerical operations for sensor processing
- `anthropic`, `openai`: AI integration
- `odrive`: Motor controller communication

**Install Poetry:**

```bash
curl -sSL https://install.python-poetry.org | python3 -
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

**Verify Poetry installation:**

```bash
poetry --version
```

**Install Python dependencies:**

```bash
cd ~/olaf
poetry install
```

This creates a virtual environment and installs all dependencies from `pyproject.toml`.

**Note:** You may see a warning about `/home/kamal/olaf/orchestrator does not contain any element` — this is expected since the orchestrator package will be created in later stories.

### 3.3 Install ROS2 Dependencies

ROS2 packages declare their dependencies in `package.xml` files. The `rosdep` tool reads these and installs required system packages automatically.

```bash
cd ~/olaf/ros2
rosdep install --from-paths src --ignore-src -r -y
```

### 3.4 Build Workspace

`colcon build` compiles all packages in the workspace. Run it from the `ros2/` directory. The `--symlink-install` flag creates symbolic links instead of copying Python files, so you can edit code without rebuilding (for Python packages).

```bash
cd ~/olaf/ros2
colcon build --symlink-install
```

**Source the workspace:**

After building, we need to source the workspace's setup script to make the built packages available. We also add this to `.bashrc` for future sessions.

```bash
source install/setup.bash
echo "source ~/olaf/ros2/install/setup.bash" >> ~/.bashrc
```

### 3.5 Verify Build

Check that the OLAF packages are recognized:

```bash
ros2 pkg list | grep -E "olaf|driver"
```

**Expected output:**
```
base_driver
head_ears_driver
neck_driver
olaf_bringup
olaf_interfaces
torso_driver
```

If packages are missing, check build logs for errors:
```bash
cat log/latest_build/*/stdout_stderr.log | grep -i error
```

### 3.6 Directory Usage Summary

| Task | Directory |
|------|-----------|
| Git operations (pull, push, status) | `~/olaf/` |
| ROS2 build (`colcon build`) | `~/olaf/ros2/` |
| ROS2 run/launch | `~/olaf/ros2/` (after sourcing) |
| ESP32 firmware development | `~/olaf/modules/{module}/firmware/` |
| Documentation | `~/olaf/docs/` |

---

## Part 4: I2C Configuration (ESP32 Modules)

I2C (Inter-Integrated Circuit) is a two-wire serial protocol perfect for connecting multiple devices to the Pi. OLAF uses I2C for the two ESP32 modules that need real-time control:

| Module | I2C Address | Purpose |
|--------|-------------|---------|
| Head ESP32 | `0x10` | Drives OLED eye displays at 60fps |
| Base ESP32 | `0x11` | Runs 200Hz balancing PID loop |

The Pi acts as the I2C master, sending commands to ESP32 slaves. This gives us 5-20ms latency which is fast enough for expression updates and velocity commands.

### 4.1 Enable I2C Interface

By default, the I2C interface is disabled on the Pi. We need to enable it in the boot configuration.

**Edit the boot config file:**

Note: Ubuntu uses `/boot/firmware/config.txt` (not `/boot/config.txt` like Raspberry Pi OS).

```bash
sudo nano /boot/firmware/config.txt
```

**Add these lines** (or uncomment if they exist):

```
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```

The first line enables the I2C interface. The second sets the bus speed to 400kHz (Fast Mode), which is faster than the default 100kHz and well within what ESP32 can handle.

**Optional: Rotate display to landscape** (if using a DSI display mounted in portrait):

```
display_rotate=3
```

Rotation values: `0`=0°, `1`=90°, `2`=180°, `3`=270° (landscape)

**Save and reboot:**
```bash
sudo reboot
```

### 4.2 Install I2C Tools

These tools let us interact with I2C devices from the command line — essential for debugging and testing.

```bash
sudo apt install -y i2c-tools python3-smbus
```

- `i2c-tools`: Command-line utilities (`i2cdetect`, `i2cget`, `i2cset`)
- `python3-smbus`: System Python bindings for I2C
- `smbus2`: Already installed via Poetry in Part 3 (pure Python I2C library used by our code)

### 4.3 Configure Permissions

By default, only root can access I2C devices. We add our user to the `i2c` group to allow access without sudo.

```bash
sudo usermod -aG i2c $USER
```

**Important:** Group changes don't take effect until you log out and back in:
```bash
exit
# SSH back in
ssh kamal@olaf.local
```

### 4.4 Verify I2C

**Check the I2C device file exists:**
```bash
ls /dev/i2c-1
```
Should show `/dev/i2c-1` without errors.

**Scan the I2C bus:**

`i2cdetect` probes all possible I2C addresses and shows which ones respond. With no devices connected, you'll see an empty grid.

```bash
i2cdetect -y 1
```

Expected output with no devices connected (all `--`):
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

**Note:** If Fusion HAT is installed, you'll see `0x17` — this is the HAT's onboard controller and is expected.

When ESP32 modules are connected and running, you'll see their addresses:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: 10 11 -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```
This shows Head at `0x10` and Base at `0x11`.

### 4.5 Test Python I2C Access

Let's verify Python can access the I2C bus:

```bash
python3 << 'EOF'
import smbus2

try:
    bus = smbus2.SMBus(1)  # Open I2C bus 1
    print("✓ I2C bus opened successfully")
    bus.close()
except PermissionError:
    print("✗ Permission denied - run: sudo usermod -aG i2c $USER")
    print("  Then log out and back in")
except FileNotFoundError:
    print("✗ I2C not enabled - check /boot/firmware/config.txt")
EOF
```

---

## Part 5: USB Serial Configuration (Waveshare Adapters)

For the neck and ear servos, we use Waveshare Bus Servo Adapters connected via USB. This approach has major advantages:
- **No soldering**: Plug-and-play USB connection
- **Integrated power**: Adapters provide regulated power to servos
- **Reliable protocol**: Standard serial communication

| Adapter | Servos | Purpose |
|---------|--------|---------|
| Waveshare Neck | 3× Feetech STS3215 | Pan, tilt, roll head movement |
| Waveshare Ears | 4× Feetech SCS0009 | 2 servos per ear for expressions |

### 5.1 Connect Adapters

Plug both Waveshare Bus Servo Adapter (A) units into USB ports on the Pi.

**Verify they're detected:**
```bash
ls /dev/ttyUSB*
```

You should see two devices like `/dev/ttyUSB0` and `/dev/ttyUSB1`. If you see `ttyACM*` instead, that's fine — some USB-serial chips use that naming.

### 5.2 Servo Libraries

Both libraries are already installed via Poetry in Part 3 — no additional installation needed:

- **pyserial**: Low-level serial port communication
- **feetech-servo-sdk**: Feetech SDK for STS/SCS servo control (position, speed, torque)

### 5.3 Configure Permissions

Like I2C, serial ports require group membership for non-root access. The `dialout` group controls serial port access.

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for this to take effect:
```bash
exit
# SSH back in
```

### 5.4 Create udev Rules for Persistent Naming

Here's a problem: USB devices can appear as `/dev/ttyUSB0` or `/dev/ttyUSB1` depending on which one the system detects first. We need consistent names so our code always talks to the right adapter.

**udev rules** let us create symbolic links with fixed names based on device attributes.

**First, get device information:**

```bash
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct|KERNELS"
```

Look for lines like:
```
ATTRS{idVendor}=="1a86"
ATTRS{idProduct}=="7523"
KERNELS=="1-1.1"
```

The `KERNELS` value (USB port path) is useful when both adapters have the same vendor/product IDs.

**Create the udev rules file:**
```bash
sudo nano /etc/udev/rules.d/99-waveshare-servos.rules
```

**Add rules based on your devices:**

If your adapters are in specific USB ports, use `KERNELS` to differentiate:
```
# Waveshare Bus Servo Adapter - Neck (adjust KERNELS to match your setup)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-1.1", SYMLINK+="waveshare_neck", MODE="0666"

# Waveshare Bus Servo Adapter - Ears
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-1.2", SYMLINK+="waveshare_ears", MODE="0666"
```

The `MODE="0666"` makes the device readable/writable by all users.

**Note:** Rules using `KERNELS` match by USB port path, not the specific board. You can swap an adapter for a different one — just plug it into the same USB port and no rule changes are needed. However, if you move an adapter to a different USB port, you'll need to update the `KERNELS` value in the rule.

**Reload udev rules:**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Verify the symlinks exist:**
```bash
ls -la /dev/waveshare_*
```

You should see symlinks pointing to the actual ttyUSB devices.

### 5.5 Test Serial Communication

Let's verify Python can open the serial ports:

```bash
python3 << 'EOF'
import serial

# Test neck adapter
try:
    ser = serial.Serial('/dev/waveshare_neck', 1000000, timeout=1)
    print("✓ Neck adapter opened at 1Mbps")
    ser.close()
except FileNotFoundError:
    print("✗ /dev/waveshare_neck not found - check udev rules")
except serial.SerialException as e:
    print(f"✗ Neck adapter error: {e}")

# Test ears adapter
try:
    ser = serial.Serial('/dev/waveshare_ears', 1000000, timeout=1)
    print("✓ Ears adapter opened at 1Mbps")
    ser.close()
except FileNotFoundError:
    print("✗ /dev/waveshare_ears not found - check udev rules")
except serial.SerialException as e:
    print(f"✗ Ears adapter error: {e}")
EOF
```

The 1000000 baud rate (1Mbps) is the default for Feetech STS series servos.

---

## Part 6: Fusion HAT Configuration

The Sunfounder Fusion HAT+ is a GPIO expansion board that provides ready-made interfaces for common robotics needs:

- **WS2812 port**: Single-wire interface for addressable RGB LEDs (24 LEDs in 3 strips of 8)
- **PWM channels**: 50Hz PWM outputs for hobby servos (kickstand mechanism)
- **I2S audio**: Digital audio output for speakers
- **I2C bridge**: Additional I2C interface

Using the Fusion HAT eliminates the need for custom PCBs or complex GPIO wiring.

### 6.1 Physical Installation

**Power off the Pi** before installing the HAT — connecting GPIO while powered can damage components.

1. Power off: `sudo shutdown -h now`
2. Carefully align the HAT with the GPIO header (pin 1 to pin 1)
3. Press down firmly until fully seated
4. Power on the Pi

### 6.2 Install Fusion HAT Libraries

The `robot-hat` library is included in OLAF's Poetry dependencies. If you completed Part 3, it's already installed.

To verify or reinstall:

```bash
cd ~/olaf
poetry install
```

**Note:** Sunfounder's official install script doesn't support Ubuntu, so we use the [PyPI package](https://pypi.org/project/robot-hat/) via Poetry instead.

**Install system dependencies for I2C/SPI access:**

```bash
sudo apt install -y python3-smbus i2c-tools
```

### 6.3 Test WS2812 LEDs

OLAF uses 24 WS2812 LEDs arranged in 3 strips of 8:
- **Interaction strip**: Shows listening/thinking/speaking states
- **Status strip**: Shows system health and errors
- **PID strip**: Visualizes balance state

```bash
python3 << 'EOF'
from robot_hat import WS2812

leds = WS2812(24)  # 24 total LEDs

# Set all LEDs to red
for i in range(24):
    leds[i] = (255, 0, 0)
leds.show()

print("✓ LEDs should now be red")
input("Press Enter to turn off...")

# Turn off
for i in range(24):
    leds[i] = (0, 0, 0)
leds.show()
EOF
```

### 6.4 Test PWM (Kickstand Servos)

The kickstand uses two standard hobby servos (model plane landing gear) controlled via PWM. Servo PWM typically runs at 50Hz with pulse widths from 1ms (0°) to 2ms (180°).

```bash
python3 << 'EOF'
from robot_hat import Servo
import time

# Initialize servo on PWM channel 0
servo = Servo(0)

print("Moving to center (0°)...")
servo.angle(0)
time.sleep(1)

input("Press Enter to move to -90°...")
servo.angle(-90)
time.sleep(1)

input("Press Enter to move to +90°...")
servo.angle(90)
time.sleep(1)

print("✓ Servo test complete")
servo.angle(0)  # Return to center
EOF
```

---

## Part 7: Hailo-8L AI Kit Setup (26 TOPS)

The Hailo-8L AI accelerator provides 26 TOPS (Tera Operations Per Second) of neural network inference power. OLAF uses this for local speech-to-text using OpenAI's Whisper model, achieving <200ms latency without cloud API calls.

**Why local STT matters:**
- **Privacy**: Voice never leaves the robot
- **Latency**: ~200ms vs 1-1.5 seconds for cloud APIs
- **Reliability**: Works offline, no internet dependency
- **Cost**: No per-request API charges

### 7.1 Hardware Overview

The Hailo-8L AI Kit is a HAT (Hardware Attached on Top) that connects to the Pi's PCIe interface via an M.2 slot. It sits on top of the Pi, with the Fusion HAT below connecting to GPIO.

```
┌─────────────────────────────┐
│   Hailo-8L AI HAT (top)     │  ← PCIe M.2 connection
│   26 TOPS neural accelerator│
├─────────────────────────────┤
│   Fusion HAT+ (middle)      │  ← GPIO connection
│   WS2812, PWM, I2S          │
├─────────────────────────────┤
│   Raspberry Pi 5 (bottom)   │
│   16GB RAM                  │
└─────────────────────────────┘
```

### 7.2 Physical Installation

**Important:** Install the Fusion HAT first, then the Hailo AI Kit on top.

1. **Power off the Pi completely**
2. Ensure Fusion HAT is already installed on GPIO header
3. Connect the Hailo-8L AI Kit:
   - Insert the M.2 module into the HAT's M.2 slot
   - Secure with the mounting screw
   - Connect the FPC ribbon cable to Pi 5's PCIe connector (if required by your kit version)
4. Stack the Hailo HAT on top (some kits use standoffs)
5. Power on the Pi

### 7.3 Install Hailo Drivers and Runtime

The Hailo software stack consists of:
- **HailoRT**: Runtime library for inference
- **TAPPAS**: Application framework with pre-built pipelines
- **Hailo Model Zoo**: Pre-compiled models including Whisper

**Add Hailo repository:**

```bash
# Install prerequisites
sudo apt install -y curl gnupg

# Add Hailo GPG key
curl -fsSL https://hailo.ai/static/hailo-repo/hailo-archive-keyring.gpg | sudo gpg --dearmor -o /usr/share/keyrings/hailo-archive-keyring.gpg

# Add Hailo repository (check Hailo docs for current URL)
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/hailo-archive-keyring.gpg] https://hailo.ai/static/hailo-repo/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/hailo.list
```

**Note:** Repository URLs may change — always check Hailo's official documentation for current instructions.

**Install HailoRT:**

```bash
sudo apt update
sudo apt install -y hailort
```

**Verify installation:**

```bash
hailortcli fw-control identify
```

Expected output shows the Hailo device info:
```
Executing on device: 0000:01:00.0
Identifying board
Control Protocol Version: 2
Firmware Version: 4.x.x
...
Board Name: Hailo-8L
```

### 7.4 Install Python Bindings

The Python bindings let us run inference from Python code:

```bash
pip install hailort
```

**Test Python access:**

```bash
python3 << 'EOF'
from hailo_platform import VDevice

try:
    # Create virtual device (connects to Hailo hardware)
    with VDevice() as device:
        print(f"✓ Connected to Hailo device")
        print(f"  Device ID: {device.device_id}")
except Exception as e:
    print(f"✗ Error: {e}")
EOF
```

### 7.5 Install Whisper Model (Phase 2 Preparation)

The Whisper model for Hailo needs to be compiled specifically for the Hailo-8L architecture. Hailo provides pre-compiled models in their Model Zoo.

**Download Whisper model (tiny or base):**

```bash
# Create models directory
mkdir -p ~/hailo_models

# Download from Hailo Model Zoo (check current URLs)
# The exact download method depends on Hailo's current distribution
# This is typically done via hailo_model_zoo package or direct download

pip install hailo_model_zoo

# List available models
hailomz list | grep whisper
```

**Note:** Whisper integration is part of Phase 2. For now, just verify the Hailo hardware is recognized and drivers are working.

### 7.6 Test Hailo Performance

Run a simple benchmark to verify the accelerator is working:

```bash
# Run built-in benchmark (if available)
hailortcli benchmark --help

# Or run a sample inference
hailortcli run-inference --help
```

### 7.7 Troubleshooting Hailo

**Device not found:**
```bash
# Check PCIe device is detected
lspci | grep Hailo
# Should show: Hailo Technologies Ltd. Hailo-8L
```

**Driver not loaded:**
```bash
# Check kernel module
lsmod | grep hailo
# Should show: hailo_pci

# If missing, load manually
sudo modprobe hailo_pci
```

**Permission denied:**
```bash
# Add user to hailo group (if exists)
sudo usermod -aG hailo $USER
# Log out and back in
```

---

## Part 8: Sunfounder Robot HAT / Fusion HAT+ Detailed Setup

Part 6 covered basic Fusion HAT installation. This section provides more detailed configuration for all the features OLAF uses.

### 8.1 Understanding the Fusion HAT+

The Sunfounder Fusion HAT+ is designed for robotics projects and provides these interfaces:

| Interface | Pins/Ports | OLAF Use |
|-----------|------------|----------|
| **WS2812** | 1 data pin | 24 indicator LEDs (3×8 strips) |
| **PWM** | 12 channels | Kickstand servos (2 channels) |
| **I2C** | SDA/SCL | Alternative I2C bus (we use Pi's native) |
| **I2S** | BCK/WS/DATA | Speaker audio output |
| **ADC** | 4 channels | Battery voltage monitoring |
| **GPIO** | Multiple | General purpose |

### 8.2 Install Sunfounder Libraries

The `robot-hat` library is included in OLAF's Poetry dependencies. If you completed Part 3 and Part 6, it's already installed.

```bash
cd ~/olaf
poetry install
```

### 8.3 Enable Required Interfaces

The Fusion HAT uses several Pi interfaces that need to be enabled:

**Edit boot config:**
```bash
sudo nano /boot/firmware/config.txt
```

**Ensure these lines are present:**

```ini
# I2C for sensors and communication
dtparam=i2c_arm=on

# SPI for some HAT features
dtparam=spi=on

# I2S audio output
dtparam=i2s=on

# PWM for servo control (hardware PWM)
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

**Reboot after changes:**
```bash
sudo reboot
```

### 8.4 WS2812 LED Configuration

WS2812 LEDs (also called NeoPixels) are addressable RGB LEDs controlled via a single data wire. The Fusion HAT has a dedicated WS2812 output.

**Test WS2812 with robot-hat library:**

```bash
python3 << 'EOF'
from robot_hat import WS2812

# Initialize 24 LEDs on the Fusion HAT's WS2812 pin
leds = WS2812(24)

# Set all LEDs to blue
for i in range(24):
    leds[i] = (0, 0, 255)  # RGB: Blue
leds.show()

print("✓ LEDs should be blue")
input("Press Enter to turn off...")

# Turn off
for i in range(24):
    leds[i] = (0, 0, 0)
leds.show()
EOF
```

**OLAF LED layout:**
```
LEDs 0-7:   Interaction strip (listening/thinking/speaking)
LEDs 8-15:  Status strip (system health, errors)
LEDs 16-23: PID strip (balance visualization)
```

### 8.5 PWM Servo Configuration

The Fusion HAT provides 12 PWM channels. OLAF uses 2 for the kickstand servos.

**Test PWM with robot-hat library:**

```bash
python3 << 'EOF'
from robot_hat import PWM
import time

# Initialize PWM channel 0 (kickstand servo 1)
servo = PWM(0)

# Standard servo: 50Hz, pulse width 500-2500μs
# robot-hat uses angle or pulse width

print("Moving servo to center (90°)...")
servo.angle(90)
time.sleep(1)

print("Moving servo to 0°...")
servo.angle(0)
time.sleep(1)

print("Moving servo to 180°...")
servo.angle(180)
time.sleep(1)

print("✓ Servo test complete")
servo.angle(90)  # Return to center
EOF
```

### 8.6 I2S Audio Configuration

The Fusion HAT supports I2S digital audio output for connecting speakers.

**Enable I2S audio:**

The I2S interface should already be enabled from the config.txt changes above. To use it:

```bash
# Check audio devices
aplay -l

# Test with a sound file
aplay /usr/share/sounds/alsa/Front_Center.wav
```

**Note:** I2S audio configuration can be complex depending on the specific DAC/amplifier used. Refer to Sunfounder documentation for your specific setup.

### 8.7 ADC for Battery Monitoring

The Fusion HAT includes ADC (Analog-to-Digital Converter) channels useful for monitoring battery voltage.

**Read ADC value:**

```bash
python3 << 'EOF'
from robot_hat import ADC

# Initialize ADC channel 0
adc = ADC(0)

# Read raw value (0-4095 for 12-bit ADC)
raw = adc.read()

# Convert to voltage (depends on your voltage divider)
# Assuming 3.3V reference and direct connection:
voltage = raw * 3.3 / 4095

print(f"ADC raw value: {raw}")
print(f"Voltage: {voltage:.2f}V")
EOF
```

### 8.8 Fusion HAT Pinout Reference

| HAT Pin | Pi GPIO | Function | OLAF Use |
|---------|---------|----------|----------|
| WS2812 | GPIO18 | LED data | Indicator LEDs |
| PWM0 | GPIO12 | PWM channel 0 | Kickstand servo 1 |
| PWM1 | GPIO13 | PWM channel 1 | Kickstand servo 2 |
| SDA | GPIO2 | I2C data | (Using Pi native) |
| SCL | GPIO3 | I2C clock | (Using Pi native) |
| I2S_BCK | GPIO18 | I2S bit clock | Audio |
| I2S_WS | GPIO19 | I2S word select | Audio |
| I2S_DATA | GPIO21 | I2S data | Audio |

**Note:** GPIO18 is shared between WS2812 and I2S. Use one or the other, not both simultaneously.

---

## Part 9: Multi-Machine ROS2 Networking

One of ROS2's strengths is seamless communication across multiple machines. Your PC and the Pi can share topics as if they were one system — you run driver nodes on the Pi (where the hardware is) and interact from your PC (where the good IDE is).

### 7.1 How It Works

ROS2 uses DDS (Data Distribution Service) for communication. When properly configured:
1. Nodes on both machines advertise their topics via UDP multicast
2. DDS discovery finds all nodes on the same domain ID
3. Topics are transparently shared across the network

```
┌─────────────┐     WiFi      ┌─────────────┐
│     PC      │◄─────────────►│     Pi      │
│ DOMAIN_ID   │   Multicast   │ DOMAIN_ID   │
│    = 42     │   Discovery   │    = 42     │
└─────────────┘               └─────────────┘
      │                             │
      └──── Shared Topics ──────────┘
         /head/expression
         /base/velocity
         /ears/emote
```

### 7.2 Verify Network Configuration

**Check the Pi's IP address:**
```bash
hostname -I
```

Note this IP — it should be on the same subnet as your PC (e.g., both in `192.168.1.x`).

**Verify environment variables:**
```bash
echo $ROS_DOMAIN_ID      # Should print: 42
echo $ROS_LOCALHOST_ONLY # Should print: 0 (or be empty)
```

Both machines must have:
- Same `ROS_DOMAIN_ID` (we use 42)
- `ROS_LOCALHOST_ONLY` set to 0 or unset

### 7.3 Test Cross-Machine Communication

This test verifies that your PC can see ROS2 messages from the Pi.

**On the Pi**, start a test publisher:
```bash
ros2 run demo_nodes_cpp talker
```

This publishes "Hello World" messages to the `/chatter` topic.

**On your PC** (with ROS2 Jazzy installed and `ROS_DOMAIN_ID=42`):
```bash
ros2 topic echo /chatter
```

You should see messages arriving from the Pi. If not, see the Troubleshooting section.

---

## Part 10: Git Configuration

Git is essential for syncing code between your PC and the Pi. When you make changes on your PC, you push to GitHub, then pull on the Pi.

### 8.1 Configure Git Identity

Git needs to know who you are for commit messages:

```bash
git config --global user.name "Your Name"
git config --global user.email "your@email.com"
```

### 8.2 Set Up SSH Key for GitHub

SSH keys let you push/pull without entering your password every time.

**Generate an SSH key:**

Ed25519 is a modern, secure key type.

```bash
ssh-keygen -t ed25519 -C "your@email.com"
```

Press Enter to accept the default file location. You can optionally set a passphrase.

**Display your public key:**
```bash
cat ~/.ssh/id_ed25519.pub
```

**Add the key to GitHub:**
1. Go to GitHub → Settings → SSH and GPG keys
2. Click "New SSH key"
3. Paste your public key
4. Save

**Test the connection:**
```bash
ssh -T git@github.com
```

Expected response: `Hi username! You've successfully authenticated...`

---

## Part 11: Display Rotation (Optional)

If you're using a DSI display (like the official Raspberry Pi display) and it's mounted in an orientation that requires rotation:

**Edit the kernel command line:**
```bash
sudo nano /boot/firmware/cmdline.txt
```

**Add at the end of the existing line** (don't create a new line):
```
video=DSI-2:800x480,rotate=270
```

Rotation values:
- `rotate=0`: No rotation (default)
- `rotate=90`: 90° clockwise
- `rotate=180`: Upside down
- `rotate=270`: 90° counter-clockwise (anticlockwise)

**Reboot to apply:**
```bash
sudo reboot
```

---

## Verification Checklist

Run through these tests to confirm everything is set up correctly:

### ✅ Ubuntu & ROS2

```bash
# Check Ubuntu version
lsb_release -a | grep "24.04"

# Check ROS2 distro
echo $ROS_DISTRO  # Should be: jazzy

# Check ROS2 works
ros2 topic list  # Should show /parameter_events and /rosout

# Check domain ID
echo $ROS_DOMAIN_ID  # Should be 42
```

### ✅ I2C

```bash
# Device file exists
ls /dev/i2c-1

# Can scan without sudo
i2cdetect -y 1

# Python can import library
python3 -c "import smbus2; print('✓ smbus2 OK')"
```

### ✅ USB Serial

```bash
# Symlinks exist
ls /dev/waveshare_*

# Python can import library
python3 -c "import serial; print('✓ pyserial OK')"
```

### ✅ Workspace

```bash
# Build succeeds
cd ~/olaf/ros2 && colcon build

# Can source workspace
source install/setup.bash
```

### ✅ Hailo AI Accelerator

```bash
# PCIe device detected
lspci | grep Hailo

# Hailo runtime works
hailortcli fw-control identify

# Python bindings work
python3 -c "from hailo_platform import VDevice; print('✓ Hailo OK')"
```

### ✅ Fusion HAT

```bash
# robot-hat library installed
python3 -c "from robot_hat import WS2812, PWM; print('✓ robot-hat OK')"
```

### ✅ Network

```bash
# Internet works
ping -c 1 google.com

# From PC: Pi is reachable
ping olaf.local
```

---

## Quick Reference

### Hardware Stack
| Layer | Component | Purpose |
|-------|-----------|---------|
| Top | Hailo-8L AI HAT | 26 TOPS neural accelerator for Whisper STT |
| Middle | Fusion HAT+ | WS2812 LEDs, PWM servos, I2S audio |
| Bottom | Raspberry Pi 5 | Main compute, ROS2, I2C, USB |

### I2C Addresses
| Module | Address | Purpose |
|--------|---------|---------|
| Head ESP32 | `0x10` | OLED eyes (2× GC9A01) |
| Base ESP32 | `0x11` | Balancing, IMU, ODrive |

### USB Serial Devices
| Device | Symlink | Purpose |
|--------|---------|---------|
| Waveshare A | `/dev/waveshare_neck` | 3× STS3215 neck servos |
| Waveshare A | `/dev/waveshare_ears` | 4× SCS0009 ear servos |

### Key Paths
```
~/olaf/                       # Git repo (everything lives here)
~/olaf/ros2/                  # ROS2 workspace (run colcon here)
~/olaf/modules/               # ESP32 firmware
~/olaf/docs/                  # Documentation
/boot/firmware/config.txt     # Boot configuration
/etc/udev/rules.d/            # udev rules
```

### Key Commands
```bash
# ROS2
ros2 topic list               # List all topics
ros2 node list                # List all nodes
ros2 topic echo /topic        # Monitor a topic
ros2 topic pub /topic ...     # Publish to a topic

# I2C
i2cdetect -y 1                # Scan I2C bus
i2cget -y 1 0x10 0x00         # Read register from device

# Deployment (on Pi)
cd ~/olaf && git pull                                      # Get latest code
cd ~/olaf/ros2 && colcon build && source install/setup.bash  # Build
```

---

## Troubleshooting

### I2C: Permission Denied

**Symptom:** `i2cdetect -y 1` shows "Permission denied"

**Fix:**
```bash
sudo usermod -aG i2c $USER
# Log out and back in (exit SSH, reconnect)
```

### USB Serial: Permission Denied

**Symptom:** Python serial open fails with permission error

**Fix:**
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### ROS2: Nodes Not Discovered Across Machines

**Symptom:** `ros2 topic list` on PC doesn't show Pi's topics

**Check 1:** Domain IDs match
```bash
# On both machines:
echo $ROS_DOMAIN_ID  # Must be 42 on both
```

**Check 2:** Localhost-only is disabled
```bash
echo $ROS_LOCALHOST_ONLY  # Should be 0 or empty
```

**Check 3:** Firewall isn't blocking
```bash
# On PC:
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

**Check 4:** Machines are on same subnet
```bash
# Both should be in same range (e.g., 192.168.1.x)
hostname -I
```

### Locale Warnings

**Symptom:** Messages like `cannot change locale`

**Fix:**
```bash
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# Log out and back in
```

### I2C: No Devices Found (When Connected)

**Symptom:** `i2cdetect` shows `--` for addresses that should have devices

**Check:**
1. **Wiring**: SDA→SDA, SCL→SCL, GND→GND between Pi and ESP32
2. **Pull-ups**: 4.7kΩ resistors from SDA to 3.3V and SCL to 3.3V
3. **Power**: ESP32 is powered and running
4. **Firmware**: ESP32 has I2C slave firmware flashed with correct address

---

## Next Steps

After completing this guide:

1. **Set up your PC** — See `docs/guides/pc-development-setup.md`
2. **Verify cross-machine ROS2** — `ros2 topic list` on PC sees Pi topics
3. **Start Epic 1** — Begin Head Module build
4. **Connect ESP32s** — Verify I2C addresses appear with `i2cdetect`

---

## Related Documentation

- **PC Setup**: `docs/guides/pc-development-setup.md`
- **Epic 0 Stories**: `docs/stories/0.1` through `0.7`
- **Tech Stack**: `docs/architecture/tech-stack.md`
- **Coding Standards**: `docs/architecture/coding-standards.md`

---

**Last Updated:** 2026-02-01
