"""
Node associated with controlling a single robot associated with team two.
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
    TODO Update docstring if CV problem solved changes.
    Team two robot, which (currently) pushes the red ball.
    """

    def __init__(self, image_topic, robot_number):
        """ 
        Initialize a single team two robot.
        """
        # Which robot number (among team one robots) this instance is.
        # Specified by command line argument.
        self.robot_number = robot_number
        # Most recent image.
        self.cv_image = None
        # Most recent image, as a binary image.
        self.binary_image = None
        # OpenCV instance associated with this node.
        self.bridge = CvBridge()
        # X mean for moments calculation.
        self.x_mean = 0
        # X values for moments calculation.
        self.x_values = 0
        # Minimum data points necessary for moments calculation.
        self.minimum_data_points = 10
        # Proportional constant for robot turn angle when tracking.
        self.kp_angle = 2
        # Default linear velocity for the robot.
        self.linear_velocity = .2

        member_image_topic = "/robot2_" + str(robot_number) + image_topic
        velocity_publisher = '/robot2_' + str(robot_number) + '/cmd_vel'
        window_name = 'TEAM TWO (RED): ROBOT ' + str(robot_number)

        rospy.Subscriber(member_image_topic, Image, self.process_image)
        self.vel_pub = rospy.Publisher(velocity_publisher, Twist, queue_size=10)
        cv2.namedWindow(window_name)

    def process_image(self, msg):
        """ 
        Stores and starts processing of incoming image data.
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.seek_red_ball()

    def seek_red_ball(self):
        """
        Search for the red ball in the robot's camera feed. If it is visible,
        drive towards it and start pushing it.
        """
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.binary_image = cv2.inRange(self.hsv_image, (0, 120, 100), (30, 256, 256))

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
        Drive towards the red ball and push it.
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_image is not None: 
                cv2.imshow('TEAM TWO (RED): ROBOT ' + str(self.robot_number), self.cv_image)
                cv2.waitKey(5)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('Team2_Robot' + sys.argv[1])
    TeamTwoActor("/camera/image_raw", int(sys.argv[1])).run()
