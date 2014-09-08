# Based on MAVLink 1.0 Messages: NAV_CONTROLLER_OUTPUT, MISSION_CURRENT
# See common.xml in MAVLink Messages

# Information regarding current waypoint target

Header header

# Mission Number (sequence)
uint16 mission_num

# Distance to WP in meters
uint16 wp_dist

# Bearing to Current target in degrees
int16 target_bearing
