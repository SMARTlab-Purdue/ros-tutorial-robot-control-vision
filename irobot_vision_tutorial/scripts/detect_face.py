#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
import sys
sys.path.append('/opt/ros/indigo/lib/python2.7/dist-packages')

def image_callback(ros_data):
       
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	gray=cv2.cvtColor(image_np,cv2.COLOR_BGR2GRAY)
	faces=face_cascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(30, 30),flags=cv2.CASCADE_SCALE_IMAGE)
	for (x,y,w,h) in faces:
		cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
		distance_from_center=((2*x+w)/2)-320
		distance_per_degree=4
		rotation_angle=distance_from_center/distance_per_degree
		pub.publish(rotation_angle)
		rospy.loginfo(rotation_angle)		
		cv2.imshow('Video',gray)
		cv2.waitKey(2)
		
	
        
      

if __name__ == '__main__':
    try:
	rospy.init_node('detect_face', anonymous=True)
	pub=rospy.Publisher('rotation_angle',Float32,queue_size=1)
	face_cascade=cv2.CascadeClassifier('/home/arabinda/catkin_ws/src/ros_seminar/scripts/haarcascade_frontalface_default.xml')
	sub = rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage, image_callback,  queue_size = 1)
	rospy.spin()
    except rospy.ROSInterruptException:
	pass
