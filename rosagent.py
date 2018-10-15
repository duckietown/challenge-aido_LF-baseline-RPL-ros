import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import numpy as np
import os
import cv2


class ROSAgent(object):
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv('HOSTNAME')
        
        # Subscribes to the action topic you'll be publishing your action messages on
        self.action_sub = rospy.Subscriber('/{}/action_topic'.format(
            self.vehicle), Twist2DStamped, self._action_cb)
        self.ik_action_sub = rospy.Subscriber('/{}/ik_action_topic'.format(
            self.vehicle), WheelsCmdStamped, self._ik_action_cb)
        # Place holder for the action, which will be read by the agent in solution.py
        self.action = np.array([0, 0])

        # Publishes onto the corrected image topic 
        # since image out of simulator is currently rectified
        self.cam_pub = rospy.Publisher('/{}/image_topic'.format(
            self.vehicle), CompressedImage, queue_size=10)
        
        # Publisher for camera info - needed for the ground_projection
        self.cam_info_pub = rospy.Publisher('/{}/camera_info_topic'.format(
            self.vehicle), CameraInfo, queue_size=1)

        # Initializes the node
        rospy.init_node('ROSTemplate')

        # 15Hz ROS Cycle - TODO: What is this number?
        self.r = rospy.Rate(15)

    def _TEMPLATE_action_publisher(self):
        """
        TODO: You need to change this!
        Random action publisher - so your submission does something
        """
        
        vl = np.random.random()
        vr = np.random.random()
        self.action = np.array([vl, vr])

    def _ik_action_cb(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])

    def _action_cb(self, msg):
        """
        Now Just for Debugging Purposes: No longer using heading/velocity - instead use vl / vr
        Callback to listen to last outputted action from lane_controller_node
        Stores it and sustains same action until new message published on topic
        """
        v = msg.v
        omega = msg.omega

        # No longer using heading/velocity - instead, vl / vr
        # self.action = np.array([v, omega])
    
    def _publish_info(self):
        """
        Publishes a default CameraInfo - TODO: Fix after distortion applied in simulator
        """

        # # TODO - You need to remove this! Triggers random action
        self._TEMPLATE_action_publisher()

        self.cam_info_pub.publish(CameraInfo())      

    def _publish_img(self, obs):
        """
        Publishes the image to the compressed_image topic, which triggers the lane following loop
        """
        img_msg = CompressedImage()

        time = rospy.get_rostime()
        img_msg.header.stamp.secs = time.secs
        img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        contig = cv2.cvtColor(np.ascontiguousarray(obs), cv2.COLOR_BGR2RGB)
        img_msg.data = np.array(cv2.imencode('.jpg', contig)[1]).tostring()
  
        self.cam_pub.publish(img_msg)