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

Get the OLAF project code onto your Pi.

```bash
cd ~                                                      # Go to home directory
git clone https://github.com/kamalkantsingh10/OLAF.git olaf  # Clone repository
cd olaf                                                   # Enter project directory
```

**Why?** This downloads all the ROS2 code, driver nodes, and configuration files you'll need. We clone it to `~/olaf` so it's in a consistent location on both your PC and Pi.


### Step 4: Install ROS2 Jazzy

Now we'll install ROS2 Jazzy on the Pi. This process takes 10-15 minutes.

**Following the official installation guide:** https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

#### What each step does:

```bash
# Update system packages
sudo apt update           # Refreshes the list of available packages
sudo apt upgrade -y      # Upgrades all installed packages to latest versions
```
**Why?** Ensures your system has the latest security patches and bug fixes before installing ROS2.

```bash
# Set up locale
locale                   # Check current locale settings
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8      # Generate US English UTF-8 locale
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8  # Set as default
export LANG=en_US.UTF-8  # Apply to current session
```
**Why?** ROS2 requires UTF-8 encoding to properly handle international characters in messages and logs.

```bash
# Add ROS2 apt repository (official method)
sudo apt install -y software-properties-common  # Tools for managing repositories
sudo add-apt-repository universe                # Enable Ubuntu's community-maintained packages
sudo apt update && sudo apt install curl -y     # Install curl for downloading files
```
**Why?** Prepares your system to download and install packages from additional sources.

```bash
# Download and install the official ROS2 repository configuration
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
**Why?** This downloads the official ROS2 repository configuration package, which adds the ROS2 package sources and GPG keys to your system. This is the recommended method as it automatically stays up-to-date.

```bash
# Update apt cache
sudo apt update  # Refresh package list to include ROS2 packages
```
**Why?** Makes the newly added ROS2 packages visible to the package manager.

```bash
# Install ROS2 Jazzy Desktop (this takes 10-15 minutes)
sudo apt install -y ros-jazzy-desktop
```
**Why?** Installs the full ROS2 Desktop package, which includes:
- Core ROS2 libraries and tools
- RViz (3D visualization tool)
- rqt (graphical tools for debugging)
- Demo packages and tutorials

```bash
# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```
**Why?**
- `colcon`: Build tool for compiling ROS2 packages
- `rosdep`: Dependency management tool that automatically installs package dependencies

```bash
# Initialize rosdep
sudo rosdep init   # Create system-wide rosdep configuration
rosdep update      # Download dependency database
```
**Why?** Sets up rosdep's database of package dependencies, which you'll need when building custom ROS2 packages.

### Step 5: Configure ROS2 Environment

Add ROS2 to your shell environment so it's available every time you log in.

#### What each step does:

```bash
# Add ROS2 environment to .bashrc
echo "" >> ~/.bashrc                                      # Add blank line for readability
echo "# ROS2 Jazzy environment" >> ~/.bashrc              # Add comment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc      # Load ROS2 environment
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc               # Set domain ID
```
**Why?**
- `source /opt/ros/jazzy/setup.bash`: Loads ROS2 into your shell PATH so you can run `ros2` commands. Without this, the system won't find ROS2 executables.
- `export ROS_DOMAIN_ID=42`: Sets your ROS2 network domain. This is like a WiFi channel - only devices with matching domain IDs can communicate. We use 42 arbitrarily (any number 0-101 works).
- Adding to `~/.bashrc` means these commands run automatically every time you log in.

```bash
# Apply changes to current session
source ~/.bashrc
```
**Why?** The `.bashrc` file only runs when you start a new terminal. This command applies the changes immediately without needing to log out and back in.

**What is ROS_DOMAIN_ID?** Think of it as a network isolation layer. If you have multiple robots or ROS2 systems on the same WiFi network, different domain IDs prevent them from interfering with each other. Your PC must use the same number (42) to communicate with the Pi.

### Step 6: Enable and Configure I2C

I2C is the protocol we use to talk to the ESP32 modules. We need to enable it and configure permissions.

#### What each step does:

```bash
# Enable I2C in boot config
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
echo "dtparam=i2c_arm_baudrate=400000" | sudo tee -a /boot/firmware/config.txt
```
**Why?**
- `dtparam=i2c_arm=on`: Enables the I2C hardware interface on the Pi's GPIO pins at boot time.
- `dtparam=i2c_arm_baudrate=400000`: Sets I2C speed to 400kHz (fast mode). Default is 100kHz (standard mode), but our ESP32 modules support the faster speed for better performance.
- These settings persist across reboots.

```bash
# Load I2C kernel module
sudo modprobe i2c-dev                        # Load the I2C driver immediately
echo "i2c-dev" | sudo tee -a /etc/modules    # Auto-load on every boot
```
**Why?** The I2C kernel module (`i2c-dev`) provides the `/dev/i2c-*` device files that programs use to communicate over I2C. `modprobe` loads it now; adding to `/etc/modules` ensures it loads automatically on future boots.

```bash
# Install I2C tools
sudo apt install -y i2c-tools
```
**Why?** Installs command-line utilities for debugging I2C:
- `i2cdetect`: Scan for I2C devices and show their addresses
- `i2cget`: Read data from I2C devices
- `i2cset`: Write data to I2C devices
- `i2cdump`: Display all registers from an I2C device

```bash
# Add your user to i2c group (allows non-root access)
sudo usermod -a -G i2c $USER
```
**Why?** By default, only root can access `/dev/i2c-*` devices. This adds your user to the `i2c` group, granting permission to use I2C without `sudo`. The change takes effect after logout/reboot.

### Step 7: Install Python I2C Library

Install the Python library that our ROS2 driver nodes use to communicate over I2C.

```bash
sudo apt install -y python3-pip    # Install pip (Python package manager)
pip3 install smbus2 --break-system-packages  # Install smbus2 library
```

**Why?** `smbus2` is a pure-Python I2C library that provides a simple interface for reading/writing to I2C devices. Our driver nodes use it to send commands to the ESP32 modules via the `/dev/i2c-1` device.

**Note:** The `--break-system-packages` flag is needed on Ubuntu 24.04 due to PEP 668 externally-managed-environment protection. This is safe for `smbus2` as it's an isolated library that doesn't conflict with system packages.

### Step 8: Reboot

Reboot to apply all configuration changes.

```bash
sudo reboot
```

**Why?** Several changes require a reboot to take effect:
- I2C hardware enablement from `/boot/firmware/config.txt`
- I2C kernel module auto-loading from `/etc/modules`
- Group membership changes (being added to the `i2c` group)

### Step 9: Verify Installation

After reboot (wait 60 seconds, then SSH back in), run these checks to confirm everything is working:

#### Check 1: ROS2 Installation
```bash
ros2 topic list
# Expected: /parameter_events, /rosout
```
**What this checks:** Verifies ROS2 Jazzy is installed and in your PATH. If you see these two topics, ROS2 is working correctly.

#### Check 2: I2C Hardware Enabled
```bash
ls /dev/i2c-*
# Expected: /dev/i2c-1
```
**What this checks:** Confirms the I2C device file exists, meaning the I2C hardware is enabled and the kernel module loaded.

#### Check 3: I2C Permissions
```bash
groups
# Expected output includes: i2c
```
**What this checks:** Verifies your user is in the `i2c` group, allowing non-root access to I2C devices.

#### Check 4: Python I2C Library
```bash
# Create a quick test script
cat > /tmp/test_i2c_open.py << 'EOF'
try:
    from smbus2 import SMBus
    bus = SMBus(1)
    bus.close()
    print("[SUCCESS] I2C bus opened successfully!")
except Exception as e:
    print(f"[FAILED] {e}")
EOF

python3 /tmp/test_i2c_open.py
# Expected: [SUCCESS] I2C bus opened successfully!
```
**What this checks:** Tests that Python can import smbus2 and open the I2C bus without permission errors.

#### Check 5: ROS2 Environment Variables
```bash
echo $ROS_DOMAIN_ID
# Expected: 42
```
**What this checks:** Confirms your `.bashrc` environment setup is working and ROS_DOMAIN_ID is set correctly.

---

**If all checks pass, your Pi is ready!** If any fail, see the Troubleshooting section below.

---

## Part 2: PC Development Setup

Your PC needs ROS2 Jazzy to communicate with the Pi over WiFi.

**Requirement:** Ubuntu 24.04 LTS (native or VM)

### Install ROS2 Jazzy

**Following the official installation guide:** https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

The PC installation is nearly identical to the Pi, except we skip I2C setup (the PC doesn't need it).

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

*See Step 4 in Part 1 for detailed explanations of what each command does.*

### Configure Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**Critical:** `ROS_DOMAIN_ID` must be **42** (matching the Pi). Different domains can't communicate with each other.

### Install Poetry (Skip if already installed)

We use Poetry for Python dependency management instead of pip. This keeps your Python environment clean and isolated.

**Check if Poetry is already installed:**
```bash
poetry --version
```

If you see a version number, skip this section. Otherwise, install Poetry:

```bash
# Install Poetry (official method)
curl -sSL https://install.python-poetry.org | python3 -

# Add Poetry to PATH
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Verify installation
poetry --version
```

**Why Poetry?** Poetry manages Python dependencies in isolation without polluting the system or breaking ROS2's Python packages. It handles virtual environments automatically.

### Clone and Build the OLAF Workspace

```bash
cd ~
git clone https://github.com/kamalkantsingh10/OLAF.git olaf
cd olaf

# Install Python dependencies using Poetry
poetry install
```

**Why?** Clones the repository and installs Python dependencies (like `smbus2` for I2C simulation, AI libraries, etc.) in an isolated Poetry virtual environment.

```bash
# Build ROS2 packages
colcon build --packages-select olaf_interfaces olaf_orchestrator
```

**What this does:** Compiles the custom ROS2 packages:
- `olaf_interfaces`: Custom message types (Expression, MotorCommand, etc.)
- `olaf_orchestrator`: Application nodes (personality coordinator, AI integration, etc.)

```bash
# Source the workspace
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/olaf/install/setup.bash" >> ~/.bashrc
```

**Why?** Makes your custom ROS2 packages available to the system. Adding to `.bashrc` means they're automatically available in new terminals.

**Using Poetry with ROS2:** When running Python scripts or ROS2 nodes that need your project dependencies, prefix commands with `poetry run`:
```bash
# Example: Run a Python script
poetry run python3 my_script.py

# Example: Launch ROS2 nodes (Poetry will ensure dependencies are available)
poetry run ros2 launch olaf_orchestrator app_nodes.launch.py
```

Alternatively, activate the Poetry virtual environment once per terminal session:
```bash
poetry shell  # Activates the virtual environment
# Now you can run commands normally without 'poetry run' prefix
ros2 launch olaf_orchestrator app_nodes.launch.py
```

### Verify Installation

```bash
ros2 topic list          # Should show: /parameter_events, /rosout
echo $ROS_DOMAIN_ID      # Should show: 42
```

**What this checks:**
- ROS2 is installed and working
- Domain ID is set correctly
- Basic ROS2 communication is functional

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
