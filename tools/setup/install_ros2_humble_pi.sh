#!/bin/bash
#
# OLAF Raspberry Pi ROS2 Humble Setup Script
# Automates installation of ROS2 Humble, I2C configuration, and development tools
#
# Usage: ./install_ros2_humble_pi.sh
#
# Requirements:
#   - Raspberry Pi OS (64-bit, Debian 12 Bookworm)
#   - Internet connection
#   - Sudo privileges
#
# This script will:
#   1. Verify system requirements
#   2. Install ROS2 Humble Desktop
#   3. Enable and configure I2C
#   4. Install Python I2C library (smbus2)
#   5. Install development tools (colcon, git)
#   6. Configure user permissions and environment

set -e  # Exit on any error

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Raspberry Pi with correct OS
check_prerequisites() {
    log_info "Checking prerequisites..."

    # Check if 64-bit ARM
    ARCH=$(uname -m)
    if [ "$ARCH" != "aarch64" ]; then
        log_error "This script requires 64-bit Raspberry Pi OS (aarch64), found: $ARCH"
        exit 1
    fi

    # Check Debian version
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$VERSION_CODENAME" != "bookworm" ] && [ "$VERSION_CODENAME" != "trixie" ]; then
            log_warn "Expected Debian 12 Bookworm or Trixie, found: $VERSION_CODENAME"
            read -p "Continue anyway? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
    else
        log_error "/etc/os-release not found. Cannot verify OS version."
        exit 1
    fi

    # Check internet connection
    if ! ping -c 1 8.8.8.8 &> /dev/null; then
        log_error "No internet connection. Please connect to WiFi/Ethernet and try again."
        exit 1
    fi

    log_info "Prerequisites check passed!"
}

# Prompt for ROS2 domain ID configuration
configure_ros2_domain() {
    log_info "Configuring ROS2 Domain ID..."
    echo ""
    echo "ROS2 Domain ID isolates your robot from other ROS2 systems on the network."
    echo "Valid range: 0-101. Default: 42"
    echo ""
    read -p "Enter ROS2 Domain ID (press Enter for default 42): " DOMAIN_ID
    DOMAIN_ID=${DOMAIN_ID:-42}

    # Validate domain ID
    if ! [[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || [ "$DOMAIN_ID" -lt 0 ] || [ "$DOMAIN_ID" -gt 101 ]; then
        log_error "Invalid domain ID. Must be between 0-101."
        exit 1
    fi

    export ROS_DOMAIN_ID=$DOMAIN_ID
    log_info "ROS2 Domain ID set to: $DOMAIN_ID"
}

# Install ROS2 Humble
install_ros2() {
    log_info "Installing ROS2 Humble..."

    # Check if ROS2 already installed
    if [ -f /opt/ros/humble/setup.bash ]; then
        log_warn "ROS2 Humble appears to be already installed."
        read -p "Reinstall? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "Skipping ROS2 installation."
            return
        fi
    fi

    # Update system
    log_info "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y

    # Install prerequisites
    log_info "Installing prerequisites..."
    sudo apt install -y curl gnupg lsb-release

    # Add ROS2 apt repository
    log_info "Adding ROS2 apt repository..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Map Debian codename to Ubuntu codename for ROS2 repositories
    # ROS2 Humble requires Ubuntu Jammy (22.04) repositories
    # Both Debian Bookworm (12) and Trixie (13) should use Jammy for Humble
    . /etc/os-release
    if [ "$VERSION_CODENAME" = "bookworm" ] || [ "$VERSION_CODENAME" = "trixie" ]; then
        UBUNTU_CODENAME="jammy"
        log_info "Using Ubuntu Jammy repository for ROS2 Humble on Debian $VERSION_CODENAME"
    else
        log_error "Unsupported Debian version: $VERSION_CODENAME"
        exit 1
    fi

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Update apt cache
    sudo apt update

    # Install ROS2 Humble Desktop
    log_info "Installing ROS2 Humble Desktop (this may take 10-15 minutes)..."
    sudo apt install -y ros-humble-desktop

    # Install development tools
    log_info "Installing ROS2 development tools..."
    sudo apt install -y python3-colcon-common-extensions
    sudo apt install -y ros-humble-rqt* python3-rosdep

    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        log_info "Initializing rosdep..."
        sudo rosdep init
    fi
    rosdep update

    log_info "ROS2 Humble installed successfully!"
}

# Configure ROS2 environment in .bashrc
configure_ros2_environment() {
    log_info "Configuring ROS2 environment in ~/.bashrc..."

    # Check if already configured
    if grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        log_warn "ROS2 environment already configured in ~/.bashrc"
    else
        echo "" >> ~/.bashrc
        echo "# ROS2 Humble environment" >> ~/.bashrc
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        log_info "Added ROS2 environment to ~/.bashrc"
    fi

    # Add ROS2 Domain ID
    if grep -q "export ROS_DOMAIN_ID=" ~/.bashrc; then
        log_warn "ROS_DOMAIN_ID already set in ~/.bashrc"
    else
        echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> ~/.bashrc
        log_info "Added ROS_DOMAIN_ID=$ROS_DOMAIN_ID to ~/.bashrc"
    fi

    # Source for current session
    source /opt/ros/humble/setup.bash
}

# Enable and configure I2C
configure_i2c() {
    log_info "Configuring I2C..."

    # Enable I2C in /boot/firmware/config.txt (or /boot/config.txt on older Pi OS)
    CONFIG_FILE="/boot/firmware/config.txt"
    if [ ! -f "$CONFIG_FILE" ]; then
        CONFIG_FILE="/boot/config.txt"
    fi

    if grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
        log_info "I2C already enabled in $CONFIG_FILE"
    else
        log_info "Enabling I2C in $CONFIG_FILE..."
        echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG_FILE" > /dev/null
    fi

    # Set I2C speed to 400kHz (default is 100kHz)
    if grep -q "^dtparam=i2c_arm_baudrate=" "$CONFIG_FILE"; then
        log_info "I2C baudrate already configured"
    else
        log_info "Setting I2C baudrate to 400kHz..."
        echo "dtparam=i2c_arm_baudrate=400000" | sudo tee -a "$CONFIG_FILE" > /dev/null
    fi

    # Load I2C kernel module
    log_info "Loading I2C kernel module..."
    sudo modprobe i2c-dev

    # Add to /etc/modules for persistent loading
    if grep -q "^i2c-dev" /etc/modules; then
        log_info "i2c-dev already in /etc/modules"
    else
        echo "i2c-dev" | sudo tee -a /etc/modules > /dev/null
    fi

    # Install I2C tools
    log_info "Installing I2C tools..."
    sudo apt install -y i2c-tools

    # Add user to i2c group for non-root access
    log_info "Adding user to i2c group..."
    sudo usermod -a -G i2c $USER

    log_info "I2C configuration complete. Reboot required for changes to take effect."
}

# Install Python I2C library
install_python_i2c() {
    log_info "Installing Python I2C library (smbus2)..."

    # Install pip if not present
    sudo apt install -y python3-pip

    # Install smbus2
    pip3 install smbus2

    log_info "smbus2 installed successfully!"
}

# Install development tools
install_dev_tools() {
    log_info "Installing development tools..."

    # Git (likely already installed)
    sudo apt install -y git

    # Check Git configuration
    if ! git config --global user.name &> /dev/null; then
        log_warn "Git user.name not configured."
        read -p "Enter your Git name: " GIT_NAME
        git config --global user.name "$GIT_NAME"
    fi

    if ! git config --global user.email &> /dev/null; then
        log_warn "Git user.email not configured."
        read -p "Enter your Git email: " GIT_EMAIL
        git config --global user.email "$GIT_EMAIL"
    fi

    log_info "Development tools installed!"
}

# Create test script to verify I2C access
create_test_script() {
    log_info "Creating I2C test script..."

    cat > /tmp/test_i2c_open.py << 'EOF'
#!/usr/bin/env python3
"""
Test script to verify I2C bus access.
This script attempts to open I2C bus 1 and confirms non-root access.
"""

try:
    from smbus2 import SMBus

    bus = SMBus(1)  # I2C bus 1
    print("[SUCCESS] I2C bus opened successfully!")
    print("[INFO] I2C device /dev/i2c-1 is accessible without sudo.")
    bus.close()

except FileNotFoundError:
    print("[ERROR] I2C device /dev/i2c-1 not found.")
    print("[FIX] Run 'sudo raspi-config' -> Interface Options -> I2C -> Enable")
    print("[FIX] Or reboot to apply kernel module changes.")
    exit(1)

except PermissionError:
    print("[ERROR] Permission denied accessing /dev/i2c-1")
    print("[FIX] Add user to i2c group: sudo usermod -a -G i2c $USER")
    print("[FIX] Then logout and login again.")
    exit(1)

except ImportError:
    print("[ERROR] smbus2 module not found.")
    print("[FIX] Install with: pip3 install smbus2")
    exit(1)
EOF

    chmod +x /tmp/test_i2c_open.py
    log_info "Test script created at /tmp/test_i2c_open.py"
}

# Print summary and next steps
print_summary() {
    echo ""
    echo "========================================"
    echo "  OLAF Raspberry Pi Setup Complete!"
    echo "========================================"
    echo ""
    log_info "ROS2 Humble: Installed"
    log_info "I2C: Enabled (400kHz)"
    log_info "smbus2: Installed"
    log_info "Development tools: Installed"
    log_info "ROS2 Domain ID: $ROS_DOMAIN_ID"
    echo ""
    log_warn "IMPORTANT: Reboot required for I2C changes to take effect."
    echo ""
    echo "After reboot, verify installation:"
    echo "  1. Check ROS2: ros2 --version"
    echo "  2. Check I2C: ls /dev/i2c-*"
    echo "  3. Test I2C access: python3 /tmp/test_i2c_open.py"
    echo "  4. Verify group membership: groups (should include 'i2c')"
    echo ""
    echo "Next steps:"
    echo "  1. Reboot: sudo reboot"
    echo "  2. Clone OLAF repository: git clone <repo_url> ~/olaf"
    echo "  3. Follow README.md for PC development setup"
    echo ""
    read -p "Reboot now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo reboot
    fi
}

# Main execution
main() {
    echo "========================================"
    echo "  OLAF Raspberry Pi ROS2 Setup"
    echo "========================================"
    echo ""

    check_prerequisites
    configure_ros2_domain
    install_ros2
    configure_ros2_environment
    configure_i2c
    install_python_i2c
    install_dev_tools
    create_test_script
    print_summary
}

# Run main function
main
