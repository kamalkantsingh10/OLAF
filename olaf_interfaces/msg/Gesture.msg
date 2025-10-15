# Gesture message for ears and neck articulation

# Neck position (degrees)
int16 neck_pan        # Pan angle: -90 to +90 degrees (left/right)
int16 neck_tilt       # Tilt angle: -45 to +45 degrees (up/down)
int16 neck_roll       # Roll angle: -30 to +30 degrees (side tilt)

# Ear positions (degrees)
int16 left_ear_base   # Left ear base servo: 0-180 degrees
int16 left_ear_tip    # Left ear tip servo: 0-180 degrees
int16 right_ear_base  # Right ear base servo: 0-180 degrees
int16 right_ear_tip   # Right ear tip servo: 0-180 degrees

uint16 duration_ms    # Movement duration in milliseconds
