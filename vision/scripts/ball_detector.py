#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('platooning_control')
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

class image_converter:

  def __init__(self):
    publish_frame = rospy.get_param('publish_frame', 'publish_frame')

    self.image_pub = rospy.Publisher(publish_frame,Image,queue_size=10)
    self.greenLower = (110, 46, 56)
    self.greenUpper = (130, 255, 255)
    self.pts = deque(maxlen=20)
    self.bridge = CvBridge()
    
    cameraFrame = rospy.get_param('camera_frame', 'camera_frame')
    print(cameraFrame)
    self.image_sub = rospy.Subscriber(cameraFrame,Image,self.callback)

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

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
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      print(radius)

      # 103 at .2 m
      # 68 at .3 m
      D = .2
      P = 103
      d = P*D/radius

      print(d)

      # only proceed if the radius meets a minimum size
      if radius > 5:
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

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)