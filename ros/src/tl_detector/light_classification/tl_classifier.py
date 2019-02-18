from styx_msgs.msg import TrafficLight
import rospy
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        area_threshold = 79

        # get red image
        red_img = image[:,:,2]
        # get the green image
        green_img = image[:,:,1]

        # get red and green areas
        red_area = np.sum(red_img == red_img.max())
        green_area = np.sum(green_img == green_img.max())

        prediction = TrafficLight.UNKNOWN

        if red_area >= area_threshold and green_area <= area_threshold:
            prediction = TrafficLight.RED
        elif red_area >= area_threshold and green_area >= area_threshold:
            prediction = TrafficLight.YELLOW if 0.8 <= red_area / green_area <= 1.2 else TrafficLight.RED
        elif green_area >= area_threshold:
            prediction = TrafficLight.GREEN
        else:
            prediction = TrafficLight.UNKNOWN

        if prediction == TrafficLight.RED:
            rospy.logwarn("RED!")

        return prediction
