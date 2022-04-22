#!/usr/bin/env python3

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import cv2
import numpy

class LaneDetection:
   def __init__(self):
       rospy.Subscriber("/image_cropped", Image, self.canny)
       rospy.Subscriber("/image_white", Image, self.white)
       rospy.Subscriber("/image_yellow", Image, self.yellow)
       self.pub_edges = rospy.Publisher("/image_edges", Image, queue_size=10)
       self.pub_white = rospy.Publisher("/image_lines_white", Image, queue_size=10)
       self.pub_yellow = rospy.Publisher("/image_lines_yellow", Image, queue_size=10)
       self.canny = None
       self.bridge = CvBridge()

   def canny(self, msg):
       cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
       edges_img = cv2.Canny(cv_img, 85, 255)
       edges_img_ros = self.bridge.cv2_to_imgmsg(edges_img, "mono8")
       self.canny = edges_img
       self.cropped_image = cv_img
       self.pub_edges.publish(edges_img_ros)
       
   def white(self, msg):
       if self.canny is not None:
           cv_img_white = self.bridge.imgmsg_to_cv2(msg, "mono8")
           cv_img_edges = self.canny
           white_edges = cv2.bitwise_and(cv_img_white, cv_img_edges)
           white_lines = cv2.HoughLinesP(white_edges, 1, numpy.pi/180, 3, minLineLength = 3, maxLineGap = 10)
           output = self.output_lines(self.cropped_image, white_lines)
           white_output_ros = self.bridge.cv2_to_imgmsg(output, "bgr8")
           self.pub_white.publish(white_output_ros)
   
   def yellow(self, msg):
       if self.canny is not None:
           cv_img_yellow = self.bridge.imgmsg_to_cv2(msg, "mono8")
           cv_img_edges = self.canny
           yellow_edges = cv2.bitwise_and(cv_img_yellow, cv_img_edges)
           yellow_lines = cv2.HoughLinesP(yellow_edges, 1, numpy.pi/180, 3, minLineLength = 3, maxLineGap = 10)
           output = self.output_lines(self.cropped_image, yellow_lines)
           yellow_output_ros = self.bridge.cv2_to_imgmsg(output, "bgr8")
           self.pub_yellow.publish(yellow_output_ros)
           
   def output_lines(self, original_image, lines):
       output = numpy.copy(original_image)
       if lines is not None:
           for i in range(len(lines)):
               l = lines[i][0]
               cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
               cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
               cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
           return output
       
if __name__ == '__main__':
    rospy.init_node('lanedetection', anonymous=True)
    LaneDetection()
    rospy.spin()
