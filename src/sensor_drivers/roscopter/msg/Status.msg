# Based on MAVLink 1.0 Messages: SYS_STATUS
# See common.xml in MAVLink Messages

Header header

# battery voltage in mV
int16 battery_voltage
# battery current in 10mA increments
int16 battery_current
# battery remaining in percentage
int8  battery_remaining

uint32 sensors_enabled
