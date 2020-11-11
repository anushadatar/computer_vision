"""
Team one image recognition node.
TODO This still needs a specific CV task
TODO this also still needs documentation 
"""
from cv_bridge import CvBridge
import cv2
import numpy as np
import rospy
import sys
import tensorflow as tf

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String

import image_classifier

class TeamOneImageRecognition():
    """
    Run image recognition using the team one robot.
    """
    def __init__(self, image_topic, robot_number):
        """
        Initialize image classification algorithm and ROS publishers and subscribers.
        """
        ## Initialize image classification algorithm and associated constants.
        # Download and cofnigure model.
        image_classifier.dowload_model()
        # Tensorflow session.
        self._session = tf.Session()
        # Initialize image classification graphdef.
        image_classifier.create_graph()
        # OpenCV bridge instance.
        self._cv_bridge = CvBridge()
        # Store most recent image store from the robot's camera feed.
        self.cv_image = None
        # Minimum score above which to consider potential classifications.
        self.score_threshold = .1
        # Number of potential classifications to consider.
        self.number_of_best_values = 5

        # Which robot number (among team one robots) this instance is.
        # Specified by command line argument.
        self.robot_number = robot_number     
        # Subscribe to the ROS topic associated with robot's camera feed.
        member_image_topic = "/robot1_" + str(robot_number) + image_topic
        rospy.Subscriber(member_image_topic, Image, self.process_image)
        # Publish to the robot's velocity topic.
        velocity_publisher = '/robot1_' + str(robot_number) + '/cmd_vel'
        self.vel_pub = rospy.Publisher(velocity_publisher, Twist, queue_size=10)
        # TODO This should probably index by robot number too
        # Debug publisher for results, use rostopic echo robot1_classifier_output to see results.
        member_debug_topic = '/robot1_' + str(robot_number) + 'classifier_output'
        self.debug_pub = rospy.Publisher(member_debug_topic, String, queue_size=1)
        # Whether or not to run in DEBUG mode.
        self.debug = True
        # TODO This does not super work as expected for showing the image, also name should change to
        # reflect chosen object to detect.
        window_name = 'TEAM ONE (BLUE): ROBOT ' + str(robot_number)        
        cv2.namedWindow(window_name)

    def process_image(self, image_msg):
        """
        Run image classification on images captured from the robot camera feed. 
        """
        self.cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        data = cv2.imencode('.jpg', self.cv_image)[1].tostring()
        softmax = self._session.graph.get_tensor_by_name('softmax:0')
        predictions = self._session.run(softmax, {'DecodeJpeg/contents:0': data})
        predictions = np.squeeze(predictions)

        # Conduct node lookup for the predictions.
        node_lookup = image_classifier.NodeLookup()
        best_values = predictions.argsort()[-self.number_of_best_values:][::-1]
        for node in best_values:
            descriptive_string = node_lookup.id_to_string(node)
            item_score = predictions[node]
            if item_score > self.score_threshold:
                rospy.loginfo('%s (score = %.5f)' % (descriptive_string, item_score))
                if self.debug:  
                    self.debug_pub.publish(descriptive_string)
            else:
                if self.debug:
                    print("Did not find any matching images.")

    def run(self):
        """
        Capture images and classify their contents using the imagenet model.
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_image is not None: 
                cv2.imshow('TEAM ONE (BLUE): ROBOT ' + str(self.robot_number), self.cv_image)
                cv2.waitKey(5)
            r.sleep()
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Team_One_Image_Recognition' + sys.argv[1])
    TeamOneImageRecognition("/camera/image_raw", int(sys.argv[1])).run()