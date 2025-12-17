# Epic 5: End-to-End Demo Script - Story Index

**Epic Goal:** Create a simple ROS2 Python script demonstrating coordinated multi-module operation to validate Phase 1 completion. The demo will sequentially activate all modules via ROS2 topic publishes: deploy kickstand, perform expressive movements (eyes, ears, head), animate heart, retract kickstand, and demonstrate balancing capability.

**Status:** Not Started
**Total Stories:** 4
**Estimated Effort:** 1-2 weeks (part-time)

---

## Stories

### [Story 5.1: Create End-to-End Demo Script](story-5.1-create-demo-script.md)
**Priority:** High | **Effort:** 8-12 hours
- Create Python script in ros2/src/olaf_bringup/scripts/phase1_demo.py
- Implement ROS2 node with rclpy
- Implement demo sequence:
  - Deploy kickstand (/neck/kickstand)
  - Blink eyes 2-3 cycles (/head_ears/eyes)
  - Move ears to perked/relaxed positions (/head_ears/ears)
  - Tilt head (pan/tilt/roll) (/neck/position)
  - Animate heart (slow → fast → slow) (/torso/heart)
  - Print demo message (/torso/print)
  - Retract kickstand (/neck/kickstand)
  - Enable balancing (/base/cmd_vel)
  - Small movement test (forward/backward) (/base/cmd_vel)
- Include appropriate delays between steps
- Include logging output for each step
- Handle graceful shutdown (Ctrl+C)
- **Outcome:** Demo script executable via `ros2 run olaf_bringup phase1_demo`

### [Story 5.2: Create Demo Launch File](story-5.2-create-demo-launch.md)
**Priority:** High | **Effort:** 4-6 hours
- Create launch file in ros2/src/olaf_bringup/launch/phase1_demo.launch.py
- Launch all 4 driver nodes (head_ears_driver, neck_driver, torso_driver, base_driver)
- Wait for drivers to initialize (2-3 second delay)
- Launch demo script (phase1_demo.py)
- Configure console output with namespace prefixes
- Ensure clean shutdown on Ctrl+C
- **Outcome:** Complete demo launchable with single command

### [Story 5.3: Test and Record End-to-End Demo](story-5.3-test-record-demo.md)
**Priority:** High | **Effort:** 8-12 hours
- Power on all 4 modules (Head+Ears, Neck, Torso, Base)
- Source ROS2 environment and verify all driver nodes functional
- Execute demo launch file
- Verify all demo steps execute successfully:
  - Kickstand deploys
  - Eyes blink visibly
  - Ears move through positions
  - Head pans, tilts, and rolls
  - Heart display shows rhythm changes
  - Thermal printer outputs message
  - Kickstand retracts
  - Robot enters balancing mode
  - Robot moves forward/backward as commanded
- Document any failures with troubleshooting steps
- Record video of full demo sequence
- Upload video for build-in-public documentation
- Tag Phase 1 milestone in Git (v1.0-phase1)
- **Outcome:** Phase 1 validated and documented with video

### [Story 5.4: Document Phase 1 Completion and Lessons Learned](story-5.4-document-phase1-completion.md)
**Priority:** High | **Effort:** 8-12 hours
- Create Phase 1 summary document (docs/guides/phase1-summary.md)
- Document total build time (weeks/hours)
- Document total cost breakdown (modules, PCBs, 3D printing, components)
- Document challenges encountered and solutions
- Document lessons learned (what worked, what didn't, what to do differently)
- Include photos of completed modules and full robot
- Finalize wiring diagrams with actual pin assignments
- Complete BOM (Bill of Materials) with supplier links
- Review and update build instructions
- Document known issues with workarounds
- Initiate Phase 2 planning (personality coordination, SLAM, AI integration)
- Update README.md with Phase 1 completion status and demo video link
- Share build-in-public post on social media
- **Outcome:** Phase 1 comprehensively documented and publicly shared

---

## Epic Completion Criteria

- [ ] Demo script created and executable
- [ ] Demo launch file created for single-command execution
- [ ] All 4 modules respond to demo commands
- [ ] Full demo sequence executes successfully:
  - [ ] Kickstand deploys
  - [ ] Eyes blink
  - [ ] Ears move
  - [ ] Head pans, tilts, rolls
  - [ ] Heart animates with rhythm changes
  - [ ] Thermal printer outputs message
  - [ ] Kickstand retracts
  - [ ] Robot balances autonomously
  - [ ] Robot moves forward/backward on command
- [ ] Demo video recorded and uploaded
- [ ] Phase 1 milestone tagged in Git (v1.0-phase1)
- [ ] Phase 1 summary document created
- [ ] Wiring diagrams finalized
- [ ] BOM complete and accurate
- [ ] Build instructions reviewed and updated
- [ ] Known issues documented
- [ ] Build-in-public post shared

---

## Key Components

- [ ] All 4 modules (Head+Ears, Neck, Torso, Base) fully operational
- [ ] ROS2 workspace with all driver nodes functional
- [ ] Demo script (phase1_demo.py)
- [ ] Demo launch file (phase1_demo.launch.py)
- [ ] Video recording equipment for demo documentation

---

## Dependencies

**Before Epic 5:**
- Epic 0: ROS2 Foundation Setup ✅
- Epic 1: Head+Ears Module Build ✅
- Epic 2: Neck Module Build ✅
- Epic 3: Torso Module Build ✅
- Epic 4: Base Module Build ✅

**After Epic 5:**
- Phase 1 complete and validated
- Ready to begin Phase 2 (Middleware & Intelligence)

---

## Notes

- **Build Order:** Epic 5 is the final epic of Phase 1 and requires all previous epics to be complete
- **Demo Purpose:** Validates that all hardware modules respond correctly to software commands, proving Phase 1 completion
- **Video Recording:** Essential for build-in-public documentation and community sharing
- **Troubleshooting:** Any failures during demo should be documented with solutions for future reference
- **Phase 1 Milestone:** Tag v1.0-phase1 in Git to mark completion
- **Phase 2 Planning:** Use lessons learned from Phase 1 to inform Phase 2 scope and approach
- **Community Sharing:** Build-in-public posts help document progress and inspire other builders

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
