# Based on MAVLink 1.0 Messages: MISSION_ITEM
# See common.xml in MAVLink Messages

# Information regarding current waypoint target

Header header

# Mission Item Number
uint16 mission_num

# Boolean to show if mission item is current
uint8 isCurrent
uint8 isAutoContinue

# Position Accuracy in meters
float32 posAcc

# Hold Time in ms
float32 holdTime

# Radius in meters for Loiter Circle.  Positive = clockwise, negative = counter
float32 loiterRadius

# Desired Yaw Angle in degrees
float32 yawFrom

# Latitude, Longitude, and Altitude
float32 x_lat
float32 y_long
float32 z_alt
