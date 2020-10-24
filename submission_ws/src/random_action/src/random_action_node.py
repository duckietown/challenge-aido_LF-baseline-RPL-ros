#!/usr/bin/env python3

import rospy
import numpy as np
import os

from duckietown_msgs.msg import WheelsCmdStamped

def continuous_publisher():
    vehicle = os.getenv("VEHICLE_NAME")
    topic = "/{}/wheels_driver_node/wheels_cmd".format(vehicle)
    vel_pub = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)
    rospy.init_node('random_action_node', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = WheelsCmdStamped()
        msg.vel_left = np.random.random()
        msg.vel_right = np.random.random()

        vel_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    continuous_publisher()


