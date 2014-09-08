#!/usr/bin/env python
import rospy
import roscopter.msg

def publish_waypt():
    rospy.init_node('waypoint_tester')

    wayptListPub = rospy.Publisher('/waypoint_list', roscopter.msg.WaypointList)

    while not rospy.is_shutdown():
        wayptListMsg = roscopter.msg.WaypointList()

        # Populate waypoint list message
        for i in range (0,20):
            wayptMsg = roscopter.msg.Waypoint()

            wayptMsg.latitude = (29.662161 + i) * 1E7
            wayptMsg.longitude = (-82.3777551 + i) * 1E7
            wayptMsg.altitude = 100 + i;
            wayptMsg.pos_acc = (10 + i) * 1000;
            wayptMsg.speed_to = 20 + i;
            wayptMsg.altitude = (30 + i) * 1000;
            wayptMsg.hold_time = (40 + i) * 1000;        # Assume defined in seconds
            wayptMsg.yaw_from = (50 + i) * 1000;

            wayptMsg.waypoint_type = roscopter.msg.Waypoint.TYPE_NAV

            wayptListMsg.waypoints.append(wayptMsg)

        rospy.sleep(1.0)
        wayptListPub.publish(wayptListMsg)

        print("Waypoints Sent")
        rospy.sleep(10.0)

if __name__ == '__main__':
    try:
        publish_waypt()
    except  rospy.ROSInterruptException:
        pass
