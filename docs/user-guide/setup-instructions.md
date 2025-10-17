# Setting Up OLAF's Development Environment

**A practical guide to the hybrid PC+Pi workflow**

---

## What You're Building

OLAF's development setup splits responsibilities between your PC and a Raspberry Pi:

- **Your PC** handles the application logic—AI reasoning, personality coordination, navigation planning. This is where you'll spend most of your time writing code.
- **Raspberry Pi** acts as the hardware bridge—it runs driver nodes that translate ROS2 messages into I2C commands for the ESP32 modules.

They communicate over WiFi using ROS2's built-in network transparency. When you're ready for production, you move everything to the Pi—**same code, zero changes**. Just a different launch file.

This guide walks through setting up both machines and getting them talking to each other.

---

## What You'll Need

**Hardware:**
- Raspberry Pi 5 (8GB recommended, 4GB works)
- MicroSD card (64GB minimum, 128GB if you want breathing room)
- Development PC running Ubuntu 24.04 (native or VM)
- Both machines on the same WiFi network

**Software (we'll install together):**
- Raspberry Pi Imager (free tool for flashing the OS)
- SSH client (built into Linux/Mac, use PuTTY on Windows)
- Git (for version control)
- ROS2 Jazzy (the robotics framework)

**What you should know:**
- Basic command-line navigation (`cd`, `ls`, `mkdir`)
- Git fundamentals (`clone`, `commit`, `push`, `pull`)
- A text editor you're comfortable with (VS Code, vim, nano—doesn't matter)

You don't need to be a Linux expert. If you can follow along with terminal commands and aren't afraid to Google error messages, you're good.

---

## Part 1: Raspberry Pi Setup

The Pi needs an operating system and some software before it can talk to hardware. We'll flash Ubuntu Server 24.04, install ROS2 Jazzy, and enable I2C (the protocol for talking to ESP32 modules).

### Step 1: Flash Ubuntu Server 24.04

**Why Ubuntu Server instead of Raspberry Pi OS?** ROS2 Jazzy has official Tier 1 support for Ubuntu 24.04 on ARM64, making installation straightforward with pre-built packages.

1. **Download Raspberry Pi Imager:**
   - https://www.raspberrypi.com/software/
   - Install it like any other program

2. **Flash Ubuntu Server:**
   - Insert your microSD card
   - Open Raspberry Pi Imager
   - **Choose Device:** Raspberry Pi 5
   - **Choose OS:** Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04 LTS (64-bit)**
     - ⚠️ **Make sure it says "24.04 LTS" (Noble Numbat)**
     - This is the official version with ROS2 Jazzy support
   - **Choose Storage:** Your microSD card

3. **Configure pre-boot settings (this is key):**

   Click the **gear icon (⚙️)** before writing. This sets up WiFi and SSH so the Pi boots ready to go.

   - **Hostname:** `olaf` (or whatever you want)
   - **Enable SSH:** ✅ Check "Use password authentication"
   - **Username:** Your choice (e.g., `pi`, `kamal`, etc. - just remember it for SSH)
   - **Password:** Something you'll remember
   - **WiFi:**
     - SSID: Your network name
     - Password: Your WiFi password
     - Country: Your region (affects WiFi frequency bands)
   - **Locale:** Your timezone and keyboard layout

   Click **Save**, then **Write**. This will erase the SD card—make sure there's nothing important on it.

4. **Boot the Pi:**
   - Eject the SD card, pop it into the Pi 5
   - Connect power
   - Wait 60-90 seconds for first boot and WiFi connection

### Step 2: SSH into the Pi

No monitor needed—we'll do everything over SSH.

**From Linux/Mac:**
```bash
ssh <username>@olaf.local
# Example: ssh kamal@olaf.local
# Or: ssh pi@olaf.local
```

**From Windows:**
- If you have Windows 10/11, SSH is built-in (use PowerShell or Command Prompt)
- Otherwise, download PuTTY: https://www.putty.org/

**Can't connect?**
- Check the Pi's power LED is solid (not blinking—means it's booted)
- Try the IP address instead of `olaf.local` (check your router's DHCP table)
- Make sure your PC is on the same WiFi network

Type `yes` when it asks about the fingerprint. You'll only see this once.

### Step 3: Clone the Repository

```bash
cd ~
git clone https://github.com/kamalkantsingh10/OLAF.git olaf
cd olaf
```


### Step 4: Install ROS2 Jazzy

Now we'll install ROS2 Jazzy on the Pi. This process takes 10-15 minutes.

**Following the official installation guide:** https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
# Update system packages
sudo apt update
sudo apt upgrade -y

# Set up locale
locale  # check for UTF-8
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository (official method)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Update apt cache
sudo apt update

# Install ROS2 Jazzy Desktop (this takes 10-15 minutes)
sudo apt install -y ros-jazzy-desktop

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 5: Configure ROS2 Environment

Add ROS2 to your shell environment so it's available every time you log in:

```bash
# Add ROS2 environment to .bashrc
echo "" >> ~/.bashrc
echo "# ROS2 Jazzy environment" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Apply changes to current session
source ~/.bashrc
```

**What is ROS_DOMAIN_ID?** It isolates your robot from other ROS2 systems on the network. We use `42` as the default. Your PC must use the same number to communicate with the Pi.

### Step 6: Enable and Configure I2C

I2C is the protocol we use to talk to the ESP32 modules. We need to enable it and configure permissions:

```bash
# Enable I2C in boot config
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
echo "dtparam=i2c_arm_baudrate=400000" | sudo tee -a /boot/firmware/config.txt

# Load I2C kernel module
sudo modprobe i2c-dev
echo "i2c-dev" | sudo tee -a /etc/modules

# Install I2C tools
sudo apt install -y i2c-tools

# Add your user to i2c group (allows non-root access)
sudo usermod -a -G i2c $USER
```

### Step 7: Install Python I2C Library

```bash
sudo apt install -y python3-pip
pip3 install smbus2
```

### Step 8: Reboot

Reboot for I2C and group membership changes to take effect:

```bash
sudo reboot
```

### Step 9: Verify Installation

After reboot, SSH back in and run these checks:

```bash
# ROS2 installed?
ros2 --version
# Expected: ros2 doctor 0.34.x (jazzy)

# I2C enabled?
ls /dev/i2c-*
# Expected: /dev/i2c-1

# Non-root I2C access?
groups
# Expected output includes: i2c

# Python I2C library works?
python3 /tmp/test_i2c_open.py
# Expected: [SUCCESS] I2C bus opened successfully!

# ROS2 domain ID set?
echo $ROS_DOMAIN_ID
# Expected: 42
```

If all checks pass, your Pi is ready!

---

## Part 2: PC Development Setup

Your PC needs ROS2 Jazzy to communicate with the Pi over WiFi.

**Requirement:** Ubuntu 24.04 LTS (native or VM)

**Install ROS2 Jazzy:**

**Following the official installation guide:** https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
# Locale setup
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository (official method)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update
```

**Configure environment:**

```bash
# Add to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**Critical:** `ROS_DOMAIN_ID` must be **42** (matching the Pi). ROS2 uses this to isolate networks—different domains can't see each other.

**Clone the repository:**

```bash
cd ~
git clone https://github.com/kamalkantsingh10/OLAF.git olaf
cd olaf
pip3 install -r orchestrator/requirements.txt
```

**Build the ROS2 workspace:**

```bash
# Build packages
colcon build --packages-select olaf_interfaces olaf_orchestrator

# Source the workspace
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/olaf/install/setup.bash" >> ~/.bashrc
```

**Verify:**

```bash
ros2 --version  # Should show jazzy
echo $ROS_DOMAIN_ID  # Should show 42
ros2 topic list  # Should show /parameter_events, /rosout
```

---

## Part 3: Test Network Communication

Time to see if the PC and Pi can talk to each other over ROS2.

**On the Pi (SSH terminal):**

```bash
cd ~/olaf
ros2 launch olaf_orchestrator drivers_only.launch.py
```

Leave this running. The Pi is now publishing status topics for the hardware drivers.

**On your PC (new terminal):**

```bash
ros2 topic list
```

**What you should see:**
```
/olaf/head/status
/olaf/ears_neck/status
/olaf/body/status
/olaf/base/status
/parameter_events
/rosout
```

If you see the `/olaf/*` topics, **it's working!** Your PC can see the Pi's nodes over WiFi.

**If you only see `/parameter_events` and `/rosout`:**
- Check `ROS_DOMAIN_ID` matches on both machines (`echo $ROS_DOMAIN_ID`)
- Verify both are on the same WiFi network
- Check firewall settings (see troubleshooting section)

**Test bidirectional communication:**

```bash
# On PC: Publish a test message
ros2 topic pub /test std_msgs/String "data: 'hello from PC'"

# On Pi (different SSH session): Echo the topic
ros2 topic echo /test
```

You should see `data: 'hello from PC'` appear on the Pi. If so, communication is fully working.

---

## Part 4: Development Workflow

Now that everything's connected, here's how you'll actually work day-to-day.

### Morning Setup (Once Per Session)

**Start Pi drivers:**
```bash
ssh pi@olaf.local
cd ~/olaf
ros2 launch olaf_orchestrator drivers_only.launch.py
```

Leave this terminal running. The Pi is now your hardware bridge—it'll stay online all day while you develop.

### On Your PC (Where You'll Work)

**Terminal 1: Edit code**
```bash
cd ~/olaf
code .  # Or vim, nano, whatever you prefer
```

**Terminal 2: Run application nodes**
```bash
ros2 launch olaf_orchestrator app_nodes.launch.py
```

**Terminal 3: Monitor ROS2 topics**
```bash
ros2 topic echo /olaf/head/expression
```

**Terminal 4: Test commands**
```bash
ros2 topic pub /olaf/head/expression olaf_interfaces/Expression \
  "{emotion_type: 1, intensity: 3, duration_ms: 2000}"
```

### Quick Iteration Cycle

1. Edit a Python file on your PC
2. Ctrl+C to stop `app_nodes.launch.py`
3. Relaunch it to test changes
4. Repeat

The Pi drivers keep running—no need to restart them. This makes iteration fast.

### ESP32 Firmware Development

You can also compile and upload ESP32 firmware from your PC:

```bash
cd modules/head
pio run  # Compile
pio run -t upload  # Upload (ESP32 connected via USB to PC)
```

The firmware runs on the ESP32. When you power it on, the Pi's driver node will communicate with it via I2C.

---

## Part 5: Deploy to Production

When you're ready to run everything on the Pi (no PC needed):

**On your PC:**
```bash
git add .
git commit -m "feat: implement personality coordinator"
git push origin main
```

**On the Pi:**
```bash
ssh pi@olaf.local
cd ~/olaf
git pull origin main

# Stop drivers_only if it's running (Ctrl+C)

# Run the full system locally
ros2 launch olaf_orchestrator olaf_full.launch.py
```

Now all nodes (drivers + application) are running on the Pi. Same code you tested on your PC—zero modifications. The only difference is they're communicating locally instead of over WiFi, which is faster (sub-millisecond latency instead of 10-50ms).

---

## Troubleshooting

### PC Can't See Pi Nodes

**Symptom:** `ros2 topic list` only shows `/parameter_events` and `/rosout`

**Check ROS_DOMAIN_ID:**
```bash
# On PC
echo $ROS_DOMAIN_ID  # Should be 42

# On Pi
echo $ROS_DOMAIN_ID  # Should also be 42
```

If they don't match, fix your `~/.bashrc` and restart terminals.

**Check network connectivity:**
```bash
ping olaf.local  # Or the Pi's IP address
```

If ping fails, your WiFi network might be isolating devices (common on guest networks or corporate WiFi).

**Check Pi nodes are running:**
```bash
# On Pi
ros2 node list
```

Should show `/olaf/head_driver`, etc. If not, the `drivers_only.launch.py` didn't start correctly.

**Check firewall (Ubuntu PC):**
```bash
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

ROS2 uses DDS (Data Distribution Service) which communicates over UDP ports 7400-7500. Some firewalls block these by default.

### Permission Denied on I2C (Pi)

**Symptom:** `PermissionError: [Errno 13] Permission denied: '/dev/i2c-1'`

**Check group membership:**
```bash
groups
```

Should include `i2c`. If not:
```bash
sudo usermod -a -G i2c $USER
sudo reboot
```

Group membership only takes effect after logout/reboot.

### I2C Device Not Found (Pi)

**Symptom:** `/dev/i2c-1` doesn't exist

**Enable I2C manually:**
```bash
sudo raspi-config
# Navigate: Interface Options → I2C → Enable
sudo reboot
```

After reboot, verify:
```bash
ls /dev/i2c-*
```

### Module Import Errors

**Symptom:** `ModuleNotFoundError: No module named 'smbus2'`

**Install dependencies:**
```bash
pip3 install -r orchestrator/requirements.txt
```

Run this in both the Pi and your PC's environment.

---

## Quick Reference

### Launch Files

| File | Run On | Purpose |
|------|--------|---------|
| `drivers_only.launch.py` | Pi | Hardware drivers only (I2C bridge) |
| `app_nodes.launch.py` | PC | Application logic (development) |
| `olaf_full.launch.py` | Pi | Complete system (production) |

### Common Commands

```bash
# Pi: Start hardware drivers
ros2 launch olaf_orchestrator drivers_only.launch.py

# PC: Start application nodes
ros2 launch olaf_orchestrator app_nodes.launch.py

# List all ROS2 nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic (see live data)
ros2 topic echo /olaf/head/expression

# Publish to a topic (test commands)
ros2 topic pub /olaf/head/expression olaf_interfaces/Expression \
  "{emotion_type: 1, intensity: 3, duration_ms: 2000}"
```

---

## Next Steps

Setup complete. Here's where to go from here:

1. **Understand the architecture:**
   - [Hybrid Development Architecture](../architecture/hybrid-development-architecture.md) - Deep dive into how PC+Pi communication works
   - [ROS2 Node Architecture](../architecture/ros2-node-architecture.md) - How nodes are structured

2. **Start building:**
   - Follow Epic 01 stories in `docs/stories/`
   - Implement your first ROS2 node (personality coordinator, AI integration)
   - Test with simulated ESP32 responses before connecting real hardware

3. **Connect hardware:**
   - Flash ESP32 firmware to modules
   - Wire I2C connections to Pi GPIO pins
   - Test driver nodes with real hardware

---

**Questions?** Check `tools/setup/README.md` for additional troubleshooting, or open a GitHub issue if you're stuck.
