#!/usr/bin/env python
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
from std_msgs.msg import MultiArrayDimension 


image_sending = [0,0]
class image_converter:

  def __init__(self):
    publish_frame = rospy.get_param('publish_frame', 'publish_frame')

    self.image_pub = rospy.Publisher(publish_frame,Image,queue_size=10)
    self.greenLower = (20, 90, 90)
    self.greenUpper = (60, 255, 255)
    self.pts = deque(maxlen=20)
    self.bridge = CvBridge()
    
    cameraFrame = rospy.get_param('camera_frame', 'camera_frame')
    print(cameraFrame)
    self.image_sub = rospy.Subscriber('device_0/sensor_1/Color_0/image/data',Image,self.callback)

  def callback(self,data):
    # pub = rospy.Publisher('output_image', Int32MultiArray, queue_size=1)  
     #rospy.init_node('publish_image', anonymous=True)
     #rate = rospy.Rate(10) #10 hz
     #while not rospy.is_shutdown():
      try:
       frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
        print(e)
      rospy.loginfo(frame)
     # global image_sending[0] = frame

      #       # if we are viewing a video and we did not grab a frame,
      # # then we have reached the end of the video
      # if frame is None:
      #     break

      # resize the frame, blur it, and convert it to the HSV
      # color space
      frame = imutils.resize(frame, width=600)
      blurred = cv2.GaussianBlur(frame, (11, 11), 0)
      hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

      # construct a mask for the color "green", then perform
      # a series of dilations and erosions to remove any small
      # blobs left in the mask
      mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
      mask = cv2.erode(mask, None, iterations=2)
      mask = cv2.dilate(mask, None, iterations=2)
      
      # find contours in the mask and initialize the current
      # (x, y) center of the ball
      cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
          cv2.CHAIN_APPROX_SIMPLE)
      cnts = imutils.grab_contours(cnts)
      center = None

      # only proceed if at least one contour was found
      if len(cnts) > 0:
          # find the largest contour in the mask, then use
          # it to compute the minimum enclosing circle and
          # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        circle_dim = [x,y,radius]
         
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print(radius)

        # 103 at .2 m
        # 68 at .3 m
        D = .2
        P = 103
        d = P*D/radius

        print(d)
        Int32MA = Int32MultiArray()  # have to assign Int32MultiArray() as a function to a variable to access the variables in it [1]
        p = rospy.Publisher('circle_dimens',Int32MultiArray,queue_size=1)
        # only proceed if the radius meets a minimum size
        if radius > 5:
          ## Publish (x,y,radius) 
          MAD = MultiArrayDimension()
          MAD.label = "circle_dimensions"
          MAD.size = 3

            # "push_back" only puts one value in the array at a time.
          Int32MA.data = circle_dim
          p.publish(Int32MA)
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
          cv2.circle(frame, (int(x), int(y)), int(radius),
          (0, 255, 255), 2)
          cv2.circle(frame, center, 5, (0, 0, 255), -1)

      # update the points queue
      self.pts.appendleft(center)

      # loop over the set of tracked points
      for i in range(1, len(self.pts)):
          # if either of the tracked points are None, ignore
          # them
        if self.pts[i - 1] is None or self.pts[i] is None:
          continue

          # otherwise, compute the thickness of the line and
          # draw the connecting lines
        thickness = int(np.sqrt(20 / float(i + 1)) * 2.5)
        cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

      # show the frame to our screen
      cv2.imshow("Frame", frame)
      cv2.imshow("Mask", mask)
      key = cv2.waitKey(1) & 0xFF

      try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
      except CvBridgeError as e:
        print(e)

     
     # pub.publish(frame)
     # rate.sleep()

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  #pub = rospy.Publisher('output_image', Int32MultiArray, queue_size=1)  
  #rospy.init_node('publish_image', anonymous=True)
  #rate = rospy.Rate(10) #10 hz
  #while not rospy.is_shutdown():
    #pub.publish(image_sending[0])
    #rate.sleep()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

    # References
    ## [1] https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226