#!/usr/bin/env python3

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import cv2

class ImageProcessing:
   def __init__(self):
       rospy.Subscriber("/image", Image, self.callback)
       self.pub_cropped = rospy.Publisher("/image_cropped", Image, queue_size=10)
       self.pub_white = rospy.Publisher("/image_white", Image, queue_size=10)
       self.pub_yellow = rospy.Publisher("/image_yellow", Image, queue_size=10)
       self.bridge = CvBridge()

   def callback(self, msg):
       kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
   
       cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
       cropped_img = cv_img[240:480, 0:640]
       cropped_img_ros = self.bridge.cv2_to_imgmsg(cropped_img, "bgr8")
       self.pub_cropped.publish(cropped_img_ros)
       
       hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
       
       filtered_img = cv2.inRange(hsv_img, (0,0,225), (180,25,255))
       dilate_img = cv2.dilate(filtered_img, kernel)
       output_img = self.bridge.cv2_to_imgmsg(dilate_img, "mono8")
       self.pub_white.publish(output_img)
       
       filtered_img = cv2.inRange(hsv_img, (28,155,155), (32,255,255))
       dilate_img = cv2.dilate(filtered_img, kernel)
       output_img = self.bridge.cv2_to_imgmsg(dilate_img, "mono8")
       self.pub_yellow.publish(output_img)
       
if __name__ == '__main__':
    rospy.init_node('imageprocessing', anonymous=True)
    ImageProcessing()
    rospy.spin()
