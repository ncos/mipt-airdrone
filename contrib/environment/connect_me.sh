#!/bin/sh
#
# Automatically try to connect to 'MyWiFi' when WiFi is enabled
#

# the output of nmcli should be in English
LC_ALL=C

# loop for a while until NetworkManager is accepting commands
# Connection name: New 802-11-wireless connection
# SSID: copter
# manual IP: 10.42.0.1

nmcli con up id 'New 802-11-wireless connection'
while [ "$(nmcli -t -f WIFI,STATE nm)" = 'enabled:disconnected' ]
do
 nmcli con up id 'New 802-11-wireless connection'
 sleep 5
done

exit 0
