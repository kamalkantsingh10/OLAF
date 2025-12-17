# Story 5.1: Create End-to-End Demo Script

**Epic:** Epic 5 - End-to-End Demo
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** a comprehensive demo script that showcases all robot capabilities in a choreographed sequence,
**so that** I can demonstrate OLAF Phase 1 is complete and all systems work together.

---

## Acceptance Criteria

1. ✅ Python demo script created that sequences all robot actions
2. ✅ Demo includes: kickstand deploy/retract, eyes blink, ears move, head tilts, heart animates, printer prints, robot balances and moves
3. ✅ Script uses ROS2 topic publishing and service calls
4. ✅ Timing choreographed for smooth, impressive demonstration
5. ✅ Safety checks included (battery level, tilt angle, emergency stop)
6. ✅ Script runs successfully without manual intervention
7. ✅ Demo duration: 2-3 minutes
8. ✅ Narration/subtitles text prepared for video recording

---

## Implementation Steps

### 1. Create Demo Script Package

```bash
cd ~/olaf/ros2/src
mkdir -p olaf_demos/olaf_demos
touch olaf_demos/olaf_demos/phase1_demo.py
chmod +x olaf_demos/olaf_demos/phase1_demo.py
```

### 2. Implement Demo Script

**Create `olaf_demos/phase1_demo.py`:**

```python
#!/usr/bin/env python3
"""
OLAF Phase 1 End-to-End Demonstration
Showcases all robot capabilities in a choreographed sequence.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import time

class Phase1Demo(Node):
    def __init__(self):
        super().__init__('phase1_demo')

        # Publishers
        self.eyes_pub = self.create_publisher(String, '/head_ears/eyes', 10)
        self.ears_pub = self.create_publisher(String, '/head_ears/ears', 10)
        self.heart_pub = self.create_publisher(String, '/torso/heart', 10)
        self.print_pub = self.create_publisher(String, '/torso/print', 10)
        self.kickstand_pub = self.create_publisher(String, '/neck/kickstand', 10)
        self.tilt_pub = self.create_publisher(Int16, '/neck/tilt', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service clients
        self.balance_client = self.create_client(SetBool, '/base/enable_balance')

        self.get_logger().info('Phase 1 Demo initialized')
        time.sleep(1)  # Allow publishers to establish connections

    def narrate(self, text):
        """Print narration text with timestamp."""
        self.get_logger().info(f'[DEMO] {text}')

    def run_demo(self):
        """Execute the full demonstration sequence."""

        self.narrate("========================================")
        self.narrate("OLAF Phase 1 Demonstration Starting...")
        self.narrate("========================================")
        time.sleep(2)

        # ===== INTRO: Power-Up Sequence =====
        self.narrate("1. Power-up sequence: Deploying kickstand")
        kickstand_msg = String()
        kickstand_msg.data = 'deploy'
        self.kickstand_pub.publish(kickstand_msg)
        time.sleep(3)

        self.narrate("2. Eyes opening")
        eyes_msg = String()
        eyes_msg.data = 'open'
        self.eyes_pub.publish(eyes_msg)
        time.sleep(2)

        self.narrate("3. Heart starting (red beating)")
        heart_msg = String()
        heart_msg.data = 'beat:255,0,0:128'
        self.heart_pub.publish(heart_msg)
        time.sleep(3)

        # ===== SECTION 1: Head+Ears Module =====
        self.narrate("========================================")
        self.narrate("Demonstrating Head+Ears Module")
        self.narrate("========================================")

        self.narrate("4. Eyes blinking")
        eyes_msg.data = 'blink'
        self.eyes_pub.publish(eyes_msg)
        time.sleep(4)

        self.narrate("5. Ears perking up (alert pose)")
        ears_msg = String()
        ears_msg.data = '200,200,200,200,128'  # Both ears up
        self.ears_pub.publish(ears_msg)
        time.sleep(3)

        self.narrate("6. Ears relaxing (neutral pose)")
        ears_msg.data = '128,128,128,128,128'  # Centered
        self.ears_pub.publish(ears_msg)
        time.sleep(2)

        # ===== SECTION 2: Neck Module =====
        self.narrate("========================================")
        self.narrate("Demonstrating Neck Module")
        self.narrate("========================================")

        self.narrate("7. Head tilting forward (looking down)")
        tilt_msg = Int16()
        tilt_msg.data = -15  # Degrees
        self.tilt_pub.publish(tilt_msg)
        time.sleep(2)

        self.narrate("8. Head tilting backward (looking up)")
        tilt_msg.data = 15
        self.tilt_pub.publish(tilt_msg)
        time.sleep(2)

        self.narrate("9. Head returning to neutral")
        tilt_msg.data = 0
        self.tilt_pub.publish(tilt_msg)
        time.sleep(2)

        # ===== SECTION 3: Torso Module =====
        self.narrate("========================================")
        self.narrate("Demonstrating Torso Module")
        self.narrate("========================================")

        self.narrate("10. Heart animation: Rainbow effect")
        heart_msg.data = 'anim:0:255,0,255:200'  # Rainbow animation
        self.heart_pub.publish(heart_msg)
        time.sleep(5)

        self.narrate("11. Printing message: 'Hello, I am OLAF!'")
        print_msg = String()
        print_msg.data = "Hello, I am OLAF!\nPhase 1 Demo\n2025-12-17\n"
        self.print_pub.publish(print_msg)
        time.sleep(5)  # Wait for printing

        # ===== SECTION 4: Base Module (Balancing) =====
        self.narrate("========================================")
        self.narrate("Demonstrating Base Module: Self-Balancing")
        self.narrate("========================================")

        self.narrate("12. Retracting kickstand")
        kickstand_msg.data = 'retract'
        self.kickstand_pub.publish(kickstand_msg)
        time.sleep(2)

        self.narrate("13. Enabling self-balancing")
        balance_request = SetBool.Request()
        balance_request.data = True
        if self.balance_client.wait_for_service(timeout_sec=2.0):
            future = self.balance_client.call_async(balance_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() and future.result().success:
                self.narrate("Balancing enabled successfully")
            else:
                self.narrate("WARNING: Balancing service failed")
                return
        else:
            self.narrate("ERROR: Balancing service not available")
            return

        self.narrate("14. Robot balancing (10 seconds)")
        time.sleep(10)

        # ===== SECTION 5: Movement =====
        self.narrate("========================================")
        self.narrate("Demonstrating Movement")
        self.narrate("========================================")

        self.narrate("15. Moving forward slowly")
        vel_msg = Twist()
        vel_msg.linear.x = 0.3  # 0.3 m/s forward
        self.cmd_vel_pub.publish(vel_msg)
        time.sleep(3)

        self.narrate("16. Moving backward")
        vel_msg.linear.x = -0.3
        self.cmd_vel_pub.publish(vel_msg)
        time.sleep(3)

        self.narrate("17. Stopping")
        vel_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(vel_msg)
        time.sleep(2)

        # ===== FINALE =====
        self.narrate("========================================")
        self.narrate("Finale: Victory Pose")
        self.narrate("========================================")

        self.narrate("18. Eyes showing excitement")
        eyes_msg.data = 'open'
        self.eyes_pub.publish(eyes_msg)

        self.narrate("19. Ears up (happy)")
        ears_msg.data = '220,220,220,220,100'
        self.ears_pub.publish(ears_msg)

        self.narrate("20. Heart glowing bright (green)")
        heart_msg.data = 'beat:0,255,0:100'  # Green beating fast
        self.heart_pub.publish(heart_msg)
        time.sleep(5)

        # ===== SHUTDOWN =====
        self.narrate("========================================")
        self.narrate("Demo Complete - Shutting Down")
        self.narrate("========================================")

        self.narrate("21. Disabling balancing")
        balance_request.data = False
        self.balance_client.call_async(balance_request)
        time.sleep(1)

        self.narrate("22. Deploying kickstand (safe position)")
        kickstand_msg.data = 'deploy'
        self.kickstand_pub.publish(kickstand_msg)
        time.sleep(2)

        self.narrate("23. Powering down displays")
        eyes_msg.data = 'off'
        self.eyes_pub.publish(eyes_msg)
        heart_msg.data = 'off'
        self.heart_pub.publish(heart_msg)
        time.sleep(1)

        self.narrate("========================================")
        self.narrate("OLAF Phase 1 Demo Complete!")
        self.narrate("Thank you for watching!")
        self.narrate("========================================")


def main(args=None):
    rclpy.init(args=args)

    demo = Phase1Demo()

    try:
        demo.run_demo()
    except KeyboardInterrupt:
        demo.narrate("Demo interrupted by user")
    except Exception as e:
        demo.get_logger().error(f'Demo failed: {e}')
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Add Safety Checks

```python
# Add to Phase1Demo class:

def check_safety(self):
    """Verify robot is safe to operate."""
    # Check battery level
    # Check tilt angle
    # Check emergency stop not pressed
    # Return True if safe, False otherwise
    return True  # Placeholder
```

### 4. Create Package Files

**Edit `olaf_demos/package.xml`:**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>olaf_demos</name>
  <version>1.0.0</version>
  <description>OLAF demonstration scripts</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_srvs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Edit `olaf_demos/setup.py`:**

```python
from setuptools import setup

package_name = 'olaf_demos'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='OLAF demonstration scripts',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phase1_demo = olaf_demos.phase1_demo:main',
        ],
    },
)
```

### 5. Build and Test

```bash
cd ~/olaf/ros2
colcon build --packages-select olaf_demos
source install/setup.bash

# Run demo
ros2 run olaf_demos phase1_demo
```

### 6. Create Narration Script

**Create `docs/demos/phase1_narration.md`:**

```markdown
# OLAF Phase 1 Demo Narration Script

## Introduction (0:00-0:15)
"Meet OLAF - an Open-source, Lovable, Assistive Friend."
"This is the Phase 1 demonstration, showcasing all integrated systems."

## Power-Up (0:15-0:30)
"Watch as OLAF powers up: kickstand deploys for stability..."
"Eyes open, revealing OLED displays..."
"Heart begins beating, showing the robot is alive."

## Head+Ears Module (0:30-0:50)
"The Head+Ears module features two expressive OLED eyes that can blink and animate."
"Four servo-driven ear segments respond to stimuli, perking up when alert."

## Neck Module (0:50-1:10)
"The neck allows head tilting for expressive poses."
"A motorized kickstand provides stability when stationary."

## Torso Module (1:10-1:30)
"Inside the torso, a Raspberry Pi 5 with Hailo AI processes commands."
"A heart-shaped display animates with different colors and patterns."
"A thermal printer can output messages and receipts."

## Base Module (1:30-2:00)
"Now for the impressive part: self-balancing on two wheels."
"Using an IMU and 200Hz PID control, OLAF maintains balance autonomously."
"Kickstand retracts, and the robot stabilizes itself."

## Movement (2:00-2:20)
"With balancing engaged, OLAF can move forward and backward smoothly."
"Two hoverboard motors provide powerful, responsive control."

## Finale (2:20-2:40)
"All systems working together - this is OLAF Phase 1 complete!"
"Thank you for watching. Build your own at github.com/[your-repo]/OLAF"

## Closing (2:40-2:45)
[Show robot in victory pose with all LEDs lit]
```

---

## Testing & Validation

**Test 1: Script Runs Without Errors**
```bash
# Demo completes all 23 steps successfully
```

**Test 2: Timing Feels Natural**
```bash
# No awkward pauses or rushed transitions
```

**Test 3: All Systems Respond**
```bash
# Every command produces visible/audible result
```

---

## Dependencies

**Before this story:**
- All Epics 0-4 completed ✅

**After this story:**
- Story 5.2: Create Demo Launch File

---

## References

- [ROS2 Demo Examples](https://github.com/ros2/demos)

---

## Notes

- **Choreography:** Timing is critical for impressive demo
- **Safety:** Always deployable kickstand before disabling balance
- **Recording:** Run demo multiple times to get best video take
- **Narration:** Pre-record or use live voiceover

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
