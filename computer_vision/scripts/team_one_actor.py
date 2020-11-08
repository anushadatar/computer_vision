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
        TODO Attribute comments
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
        self.debug = True

        member_image_topic = "/robot1_" + str(robot_number) + image_topic
        rospy.Subscriber(member_image_topic, Image, self.process_image)
        print(member_image_topic)
        velocity_publisher = '/robot1_' + str(robot_number) + '/cmd_vel'
        self.vel_pub = rospy.Publisher(velocity_publisher, Twist, queue_size=10)
        window_name = 'TEAM ONE (BLUE): ROBOT ' + str(robot_number)
        cv2.namedWindow(window_name)
        if self.debug:
            cv2.setMouseCallback(window_name, self.process_mouse_event)

    def process_mouse_event(self, event, x,y,flags,param):
        """ 
        Process mouse events so that you can see the color values
        associated with a particular pixel in the camera images. Useful in debug mode.
        """
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)


    def process_image(self, msg):
        """ 
        TODO better docstring
        Stores incoming image data and processes it when possible.
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.seek_blue_ball()

    def seek_blue_ball(self):
        """
        TODO This is using the track image code from neato soccer, we would want to seed with our 
        like proposed algorithm and values/
        """
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.binary_image = cv2.inRange(self.hsv_image, (100, 120, 110), (130, 256, 256))

        moments = cv2.moments(self.binary_image, binaryImage=True)
        if moments['m00'] >= self.minimum_data_points:
            shape = self.cv_image.shape
            self.x_values = moments['m10'] / moments['m00']
            self.x_mean = (self.x_values / shape[1]) - 0.5
            # Drive the motors based on the proximity of the item.
            angular_velocity = -self.kp_angle * self.x_mean
            self.vel_pub.publish(Twist(linear=Vector3(x=self.linear_velocity), angular=Vector3(z=angular_velocity)))
            if self.debug:
                print("Detected object and moving towards it.")


    def run(self):
        """
        TODO
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_image is not None: 
                cv2.imshow('TEAM ONE (BLUE): ROBOT ' + str(self.robot_number), self.cv_image)
                cv2.waitKey(5)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('Team1_Robot' + sys.argv[1])
    TeamOneActor("/camera/image_raw", int(sys.argv[1])).run()