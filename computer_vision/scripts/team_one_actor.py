"""
TODO Block comment
"""
import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

import sys

class TeamOneActor(object):
    """
    Tracks objects in the view of the robot.
    """

    def __init__(self, image_topic, robot_number):
        """ 
        TODO  docstring
        """
        self.robot_number = robot_number
        self.cv_image = None
        self.binary_image = None
        self.bridge = CvBridge()

        member_image_topic = "/robot1_" + str(robot_number) + image_topic
        print(member_image_topic)

        rospy.Subscriber(member_image_topic, Image, self.process_image)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        cv2.namedWindow('TEAM ONE: ROBOT ' + str(robot_number))

    def process_image(self, msg):
        """ 
        TODO better docstring
        Stores incoming image data and processes it when possible.
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def run(self):
        """
        TODO
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_image is not None: 
                cv2.imshow('TEAM ONE: ROBOT ' + str(self.robot_number), self.cv_image)
                cv2.waitKey(5)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('Team1_Robot' + sys.argv[1])
    TeamOneActor("/camera/rgb/image_raw", int(sys.argv[1])).run()