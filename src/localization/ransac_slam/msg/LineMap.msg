# This message holds a collection of lines

# Time of sensor data acquisition, coordinate frame ID.
Header header

int64   number                  # number of lines
geometry_msgs/Point32[] dfdirs  # vectors ortogonal to the corresponding line with the end on this line
geometry_msgs/Point32[] ldirs   # vectors collinear to the corresponding line
