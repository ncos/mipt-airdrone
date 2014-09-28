# This message holds a collection of lines and passages

# Time of sensor data acquisition, coordinate frame ID.
Header header

int64   number                  # number of lines
geometry_msgs/Point32[] dfdirs  # vectors ortogonal to the corresponding line with the end on this line
geometry_msgs/Point32[] ldirs   # vectors collinear to the corresponding line

int64   pnumber                 # number of passages
geometry_msgs/Point32[] pleft   # left point of the passage
geometry_msgs/Point32[] prght   # right point of the passage
