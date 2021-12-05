#!/usr/bin/env python3
# Capturing image from the color and depth ros topics of realsense camera
# Adapted from http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.color_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
    self.image_count = 0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except:
      cv_image = self.bridge.imgmsg_to_cv2(data)
    
    if self.image_count < 2 :
        print('saving image')
        self.image_count = self.image_count + 1
        cv2.imwrite("/home/david/stuff/test.jpg", cv_image)        

    print(type(cv_image)) # numpy
    print(cv_image.shape) 

def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()