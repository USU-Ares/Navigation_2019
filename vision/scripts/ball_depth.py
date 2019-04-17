#!/usr/bin/env python  
#[2] Have to include so that when using rosrun, it knows to use
## python to interpret what the python references are.
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import time
import tf
from std_msgs.msg import Int32MultiArray 

class Valid_Depth:
    def __init__(self): # [1] self means that the values are only related to 
                        ## to this reference to the class.
        self.depth_bounding_box = []
        self.bridge = CvBridge()
        
        # Read in the (x,y) pixel position of the center of the tennis ball and the radius
        self.circle_values = rospy.Subscriber('circle_dimens', Int32MultiArray, self.callback1)     

    def callback1(self,data1):

        circle_v = data1.data

         # Want to set up a min and max pixel locations in the x and y direction of 
         ## the center of the tennis ball to assign bounding box values to 
         ## "depth_bounding_box".
        R_min = circle_v[2] - 2
        x_min = circle_v[0] - R_min
        x_max = circle_v[0] + R_min
        y_min = circle_v[1] - R_min
        y_max = circle_v[1] + R_min

        self.depth_bounding_box = [x_min,x_max,y_min,y_max,circle_v[0],circle_v[1]]

         # Read in depth image
        self.depth_image = rospy.Subscriber('device_0/sensor_0/Depth_0/image/data', Image, self.callback2)

    def callback2(self,data2):
        depth_frame = self.bridge.imgmsg_to_cv2(data2,"32FC1")
        rospy.loginfo(depth_frame)
        depth_value = depth_frame[self.depth_bounding_box[5]][self.depth_bounding_box[4]]
        print(depth_value)

def main(arg):
    init_class = Valid_Depth()
    rospy.init_node('ball_depth', anonymous=True)
   
    try:
        rospy.spin()
    except KeyboardInterrupt:    # if at any point (as the code "spins",or repeats)
                                 ## a key on the keyboard is hit in the command
                                 ## window (which KeyboardInterrupt is a built in sys.argv)
                                 ## then it will then jump to the "except" statement and 
                                 ## execute what is below it.
        print("Closing")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

# References
## [2] https://stackoverflow.com/questions/16069816/getting-python-error-from-cant-read-var-mail-bio