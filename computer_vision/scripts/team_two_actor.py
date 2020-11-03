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

class TeamTwoActor(object):
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

        self.x_mean = 0
        self.x_values = 0
        self.minimum_data_points = 10
        self.kp_angle = 2
        self.linear_velocity = .2

        member_image_topic = "/robot2_" + str(robot_number) + image_topic
        print(member_image_topic)

        rospy.Subscriber(member_image_topic, Image, self.process_image)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        cv2.namedWindow('TEAM TWO: ROBOT ' + str(robot_number))
        

    def process_image(self, msg):
        """ 
        TODO better docstring
        Stores incoming image data and processes it when possible.
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.track_image_with_matched_color(msg)

    def track_image_with_matched_color(self, msg):
        """
        TODO This is using the track image code from neato soccer, we would want to seed with our 
        like proposed algorithm and values/
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.binary_image = cv2.inRange(self.hsv_image, (40, 120, 100), (65, 256, 256))

        moments = cv2.moments(self.binary_image, binaryImage=True)
        if moments['m00'] >= self.minimum_data_points:
            shape = self.cv_image.shape
            self.x_values = moments['m10'] / moments['m00']
            self.x_mean = (self.x_values / shape[1]) - 0.5
            # Drive the motors based on the proximity of the item.
            angular_velocity = -self.kp_angle * self.x_mean
            self.vel_pub.publish(Twist(linear=Vector3(x=self.linear_velocity), angular=Vector3(z=angular_velocity)))

    def run(self):
        """
        TODO
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_image is not None: 
                cv2.imshow('TEAM TWO: ROBOT ' + str(self.robot_number), self.cv_image)
                cv2.waitKey(5)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('Team2_Robot' + sys.argv[1])
    TeamTwoActor("/camera/image_raw", int(sys.argv[1])).run()