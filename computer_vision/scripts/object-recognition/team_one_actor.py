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

class ImageRecognitionTestNode():
    """
    TODO
    """
    def __init__(self, image_topic, robot_number):
        """
        TODO docstring
        """
        # TODO attribute comments
        image_classifier.dowload_model()
        self._session = tf.compat.v1.Session()
        image_classifier.create_graph()

        self._cv_bridge = CvBridge()
        self.cv_image = None
        self.score_threshold = .1
        self.number_of_best_values = 5

        self.robot_number = robot_number        
        image_topic = image_topic
        member_image_topic = "/robot1_" + str(robot_number) + image_topic
        rospy.Subscriber(member_image_topic, Image, self.process_image)
        velocity_publisher = '/robot1_' + str(robot_number) + '/cmd_vel'
        self.vel_pub = rospy.Publisher(velocity_publisher, Twist, queue_size=10)
        # TODO This should probably index by robot number too
        # Debug publisher for results, use rostopic echo robot1_classifier_output to see results.
        self.debug_pub = rospy.Publisher('/robot1_classifier_output', String, queue_size=1)
        self.debug = True
        # TODO This does not super work as expected.
        window_name = 'TEAM ONE (BLUE): ROBOT ' + str(robot_number)        
        cv2.namedWindow(window_name)


    def process_image(self, image_msg):
        """
        TODO 
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
                # TODO Here is probably where we want to add the image recognition words
                # we want to track and the controller logic.
                # We can find that logic in the color-detection/team-one-actor.
            else:
                if self.debug:
                    print("Did not find any matching images.")

    def run(self):
        """
        Drive towards the blue ball and push it.
        TODO
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_image is not None: 
                # TODO This window does not quite show up as expected.
                cv2.imshow('TEAM ONE (BLUE): ROBOT ' + str(self.robot_number), self.cv_image)
                cv2.waitKey(5)
            r.sleep()
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Team_One_Image_Recognition' + sys.argv[1])
    ImageRecognitionTestNode("/camera/image_raw", int(sys.argv[1])).run()