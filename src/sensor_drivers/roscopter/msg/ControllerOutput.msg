# Based on MAVLink 1.0 Messages: NAV_CONTROLLER_OUTPUT
# See common.xml in MAVLink Messages

# Outputs of the ArduPilot Mega controller.

# Current Desired Roll, Pitch, and Yaw in degrees
float32 nav_roll
float32 nav_pitch
int16 nav_bearing

# Current error in altitude, airspeed, and crosstrack
float32 alt_error
float32 air_spd_error
float32 xtrack_error
