# ROS Image Processing
This package includes ROS nodes that subscribe to external images and use OpenCV functions to identify certain lanes. Hough lines are then drawn over the edges of these lanes for the robot to autonomously follow.
* ImageProcessing.py isolates the white and yellow lanes individually and publishes each to their own respective topics.
* LaneDetection.py subscribes to the images published by ImageProcessing. It then runs OpenCV Canny edge detection on the images to detect edges, draws Hough lines over the detected edges, and publishes the white and yellow lines to their own respective topics.
<br/>
<img src="https://github.com/ChrisAurelio/ROSImageProcessing/blob/main/Examples/original.jpg" width=65%>

<img src="https://github.com/ChrisAurelio/ROSImageProcessing/blob/main/Examples/white.jpg" width=65%>

<img src="https://github.com/ChrisAurelio/ROSImageProcessing/blob/main/Examples/yellow.jpg" width=65%>

<img src="https://github.com/ChrisAurelio/ROSImageProcessing/blob/main/Examples/whitelines.jpg" width=65%>

<img src="https://github.com/ChrisAurelio/ROSImageProcessing/blob/main/Examples/yellowlines.jpg" width=65%>
