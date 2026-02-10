# Direct ear angle control for OLAF ears module
# All angles in degrees relative to center (0 = straight up)
# Pan: 0-90 (outward), Tilt: -90 to +110

float32 left_pan    # Left ear base rotation in degrees (0=center, +90=outward)
float32 left_tilt   # Left ear angle in degrees (-90=back, +110=forward)
float32 right_pan   # Right ear base rotation in degrees (0=center, +90=outward)
float32 right_tilt  # Right ear angle in degrees (-90=back, +110=forward)
float32 speed_pct   # Speed adjustment fraction (-0.3 to +0.3), 0.0 = base speed
