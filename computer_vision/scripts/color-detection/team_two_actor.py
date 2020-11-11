"""
Node associated with controlling a team two actor, which finds the red ball
and attempts to shoot it into the goal.
"""
import rospy
from copy import deepcopy
import cv2
import numpy as np
import sys

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from sensor_msgs.msg import Image

class TeamTwoActor(object):
    """
    Attempt to shoot the red ball into a goal on the field.
    """
    def __init__(self, image_topic, robot_number):
        """
        Initialize a single team one robot.
        """
        # Which robot number (among team one robots) this instance is.
        # Specified by command line argument.
        self.robot_number = robot_number
        # Most recent image.
        self.cv_image = None
        self.cv_annotated = None

        # Most recent image, as a binary ball image.
        self.ball_binary_image = None
        # Most recent image, as a binary goal image.
        self.goal_binary_image = None
        # OpenCV instance associated with this node.
        self.bridge = CvBridge()
        # X mean for moments calculation.
        self.x_mean = 0
        # X values for moments calculation.
        self.x_values = 0
        # Minimum data points necessary for moments calculation.
        self.minimum_data_points = 3
        # Proportional constant for robot turn angle when tracking.
        self.kp_angle = 1.2
        # Default linear velocity for the robot.
        self.linear_velocity = .3
        # Flag associated with enabling and disabling debug prints.
        self.debug = True
        
        #Initialize publishers, subscriber, and image window.
        member_image_topic = "/robot2_" + str(robot_number) + image_topic
        velocity_publisher = '/robot2_' + str(robot_number) + '/cmd_vel'
        window_name = 'TEAM TWO (RED): ROBOT ' + str(robot_number)
        rospy.Subscriber(member_image_topic, Image, self.process_image)
        self.vel_pub = rospy.Publisher(
            velocity_publisher, Twist, queue_size=10)
        cv2.namedWindow(window_name)

    def process_image(self, msg):
        """
        Stores and starts processing of incoming image data.
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Generate copy for annotations, and binaries for detections
        self.cv_annotated = self.cv_image
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # Goal is yellow, ball is either red or blue, depending on the player
        self.goal_binary_image = cv2.inRange(
            self.hsv_image, (20, 100, 100), (30, 256, 256))
        self.ball_binary_image = cv2.inRange(
            self.hsv_image, (0, 120, 100), (30, 256, 256))

        # Make annotations, determine flag for shot on goal.
        self.annotate_contours()

        # Navigate robot towards a shot on goal.
        self.seek_binary(self.ball_binary_image, self.goal_binary_image)

    def annotate_contours(self):
        """
        Visualizes detected objects on bot camera view using binary image.
        """
        goal_contours = cv2.findContours(
            self.goal_binary_image,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        goal_contours = goal_contours[0] if len(
            goal_contours) == 2 else goal_contours[1]
        for cntr in goal_contours:
            x, y, w, h = cv2.boundingRect(cntr)
            if w * h > 100:
                cv2.rectangle(self.cv_annotated, (x, y),
                              (x + w, y + h), (0, 0, 255), 2)

        ball_contours = cv2.findContours(
            self.ball_binary_image,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        ball_contours = ball_contours[0] if len(
            ball_contours) == 2 else ball_contours[1]
        for cntr in ball_contours:
            x, y, w, h = cv2.boundingRect(cntr)
            if w * h > 100:
                cv2.rectangle(self.cv_annotated, (x, y),
                              (x + w, y + h), (0, 255, 0), 2)

    def seek_binary(self, ball_binary, goal_binary):
        """
        Attempt to drive the object detected from ball_binary into the center of the goal_binary detection.
        """
        # Calculate moments of ball binary and goal binary
        ball_moments = cv2.moments(ball_binary, binaryImage=True)
        goal_moments = cv2.moments(goal_binary, binaryImage=True)

        if ball_moments['m00'] >= self.minimum_data_points and goal_moments['m00'] >= self.minimum_data_points:
            shape = self.cv_image.shape
            # Find x positions of ball and goal
            self.ball_x_values = ball_moments['m10'] / ball_moments['m00']
            self.ball_x_mean = (self.ball_x_values / shape[1]) - 0.5
            self.goal_x_values = goal_moments['m10'] / goal_moments['m00']
            self.goal_x_mean = (self.goal_x_values / shape[1]) - 0.5

            # Drive at an angle to the ball until its parallax distance from
            # the goal is below a threshold
            angular_velocity = 0.3 * (self.ball_x_mean - self.goal_x_mean)

            if abs(self.goal_x_mean - self.ball_x_mean) < 0.2:
                # Ball and goal are sufficiently lined up in frame for a shot
                # on goal
                angular_velocity = 2 * (self.ball_x_mean + self.goal_x_mean)

            self.vel_pub.publish(
                Twist(
                    linear=Vector3(
                        x=self.linear_velocity), angular=Vector3(
                        z=angular_velocity)))
        else:
            # We've lost the ball or goal, rotate slowly until we can get both in frame again
            # Consider the mission failed if both are not attainable in the
            # same frame
            self.vel_pub.publish(
                Twist(
                    linear=Vector3(
                        x=0), angular=Vector3(
                        z=0.5)))

    def run(self):
        """
        Drive towards the blue ball and push it.
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_annotated is not None:
                cv2.imshow('TEAM TWO (RED): ROBOT ' +
                           str(self.robot_number), self.cv_annotated)
                cv2.waitKey(5)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('Team2_Robot' + sys.argv[1])
    TeamTwoActor("/camera/image_raw", int(sys.argv[1])).run()
