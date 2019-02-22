#!/usr/bin/env python
"""ROS node that publishes the pozyx 1d position """

import pypozyx
import rospy
import math
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan

remote_id = None

def pozyx_1d_pub():
    rospy.loginfo("POZYX 1D NODE INIT")
    pub_ls = rospy.Publisher('/paint/pozyx/laser_scan', LaserScan, queue_size=1)
    rospy.init_node('pozyx_1d_pub')

    #rate
    rate = rospy.Rate(40)

    # read parameters
    names = str(rospy.get_param("~names_list")).split(';')
    names = [int(i, 0) for i in names]
    rospy.loginfo("Pozyx anchor names list: " + str(names))
    
    N = len(names)
    rospy.loginfo("Anchors number: " + str(N))
    
    ls = LaserScan()
    ls.ranges = [0.0] * N
    timestamps = [0.0] * N

    serial_port = rospy.get_param("~serial")
    rospy.loginfo("Pozyx serial port: " + serial_port)

    # end parameters reading


    try:
        #TODO: serial port to parameters
        pozyx = pypozyx.PozyxSerial(serial_port)
    except:
        rospy.logerr("Pozyx not connected.")
        return

    device_range = pypozyx.DeviceRange()

    err_count = 0

    while not rospy.is_shutdown():
       
        for i in range(0, N):
            res = pozyx.doRanging(names[i], device_range, remote_id=remote_id)
            if (res == pypozyx.POZYX_SUCCESS):
                err_count = 0
                ls.ranges[i] = device_range.distance / 1000.0
                timestamps[i] = device_range.timestamp
		        #rospy.loginfo("NAME: " + str(names[i]) +", " + str(ranges[i]))
            if (res == pypozyx.POZYX_TIMEOUT):
                ls.ranges[i] = -1
            if (res == pypozyx.POZYX_FAILURE):
                ls.ranges[i] = -2
                err_count = err_count + 1
                rospy.logerr("Pozyx Error DoRanging! Result=" + str(res))
            if (err_count > 10):
                rospy.logerr("Pozyx disconnected.")
                return
            ls.header.stamp = rospy.Time.now()
            pub_ls.publish(ls)
            rate.sleep()

if __name__ == '__main__':
    try:
        pozyx_1d_pub()
    except rospy.ROSInterruptException:
        pass
