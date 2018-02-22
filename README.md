ROS tutorial by Purdue SMART lab: iRobot Create2 teleoperation and Computer Vision based object detection for mobile robot control.

The wiki page of this repository also has the tutorial instructions: <https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki>

# 1. Objectives
This ROS tutorial provides an overview of teleoperating (control) an iRobot create2 (roomba) mobile robot. Then, we look at ways to make use of the robot to track a ball/person using openCV algorithms. For those who do not have a creat2 robot, we also show a case of controlling a turtlebot in simulation (instead of iRobot control) in this tutorial. 
Additionally, this tutorial will help you learn some basics of computer vision and robot control methods in ROS.

Demonstration video:
Please watch the videos below to get an idea of what is expected to be achieved by this tutorial.

<https://www.youtube.com/watch?v=ZyD-bbF6ts4>

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/ZyD-bbF6ts4/0.jpg)](https://www.youtube.com/watch?v=ZyD-bbF6ts4)

<https://www.youtube.com/watch?v=ZyD-bbF6ts4>

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/-P3i7L_g1OM/0.jpg)](https://www.youtube.com/watch?v=-P3i7L_g1OM)



# 2. Credits
This tutorial is prepared by Arabinda Samantaray (samantar@purdue.edu) and Shaocheng Luo (scluo@purdue.edu).
We acknowledge the following sources which were used to prepare this tutorial:

- http://wiki.ros.org/ROS/Tutorials

- http://milq.github.io/install-opencv-ubuntu-debian/

- https://docs.opencv.org/3.3.0/d7/d8b/tutorial_py_face_detection.html

- https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

- ROS create_autonomy package summary <http://wiki.ros.org/create_autonomy>    

- ROS driver for iRobot Create 1 and 2 <https://github.com/autonomylab/create_autonomy>    

- ROS joy <http://wiki.ros.org/joy>

# 3. Prerequisites
A basic background of using Linux-based OS, ROS and OpenCV will be required to understand the following tutorial. If you need some help, please visit:

- http://wiki.ros.org/ROS/Tutorials

- http://files.ubuntu-manual.org/manuals/getting-started-with-ubuntu/14.04e2/en_US/screen/Getting%20Started%20with%20Ubuntu%2014.04%20-%20Second%20edition.pdf

- https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_tutorials.html

- http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

This tutorial was tested successfully in Ubuntu 14.04, ROS-indigo with OpenCV 3.0.0.

# 4. Installation of relevant dependencies/packages

###  Install ROS
We used ROS Indigo in this tutorial. But the tutorial could work even in ROS Kinetic/Lunar, although we did not validate it in Kinetic Kame. 

We recommend you install _ros-indigo-desktop-full_ so that you have all the necessary packages. The full package comes with Gazebo 2.2 as default. We recommend using a desktop PC or a laptop with Ubuntu 14.04+. 

### Installing iRobot Create2 (Roomba 600) Driver
The ROS driver for Create2 robot is provided by the Autonomy lab at SFU, Canada. 
Install the ROS driver for the robot using the following instructions. For detailed driver installation instructions, please look at <https://github.com/AutonomyLab/create_autonomy>.

### Workspace Setup

If you want to create a new catkin workspace for the robot driver, then 
    ``` bash
    $ cd ~
    $ mkdir -p create_ws/src  
    $ cd ~/create_ws
    $ catkin_make  
    ```
Else, use an existing ROS workspace for the next step.

Clone the repository of the robot driver 
    ``` bash
    $ cd ~/create_ws/src
    $ git clone https://github.com/AutonomyLab/create_autonomy.git  
    ```
  
Build/Compile  
    ``` bash
    $ cd ~/create_ws
    $ catkin_make
    ```

### Check USB Permissions

1. In order to connect to Create over USB, ensure your user is in the dialout group
    ``` bash
    $ sudo usermod -a -G dialout $USER
    ```

2. Logout of the OS and log in again for the permission to take effect

### Teleoperation Twist Keyboard Package Installation    

Install the package of Teleoperation Twist Keyboard    

``` bash
    $ sudo apt-get install ros-indigo-teleop-twist-keyboard
```

###  Install OpenCV
- The easiest way to install openCV (if you do not have it installed already) is to do:
``` bash
sudo apt-get install libopencv-dev python-opencv 
```
- If the above resulted in errors, then perhaps the following resource might be useful: <http://milq.github.io/install-opencv-ubuntu-debian/>

###  Install USB camera drivers

- In your home workspace\src

``` sudo apt-get install ros-indigo-usb-cam ```

### Download HaarCascade file for image processing
 - Download ``` haarcascade_frontalface_default.xml``` file from <https://github.com/opencv/opencv/tree/master/data/haarcascades> and save it in your designated ```ros_package/scripts``` folder, which contains the script that uses the haar cascade.

### Install turtle simulator (if no physical robot is available)
``` sudo apt-get install ros-indigo-turtlesim ```


# 5. Tutorial


## 5.1 iRobot control with a keyboard 

![irobot](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/w3_o1.png)

### ROS Launch Files   

First, run a roscore, which will act as master and establishes connections between all ROS nodes.

``` roscore ```

For Create2 robot (Roomba 600/700 series), launch the ROS drive using the following command:    

``` bash

    $ roslaunch ca_driver create_2.launch

```
![irobot](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/w3_or2.png)

### Check the ROS Topics list to see if the robot drivers are publishing the state such as the odometry output in /odom, bumper sensor readings, etc.

``` bash

    $ rostopic
    $ rostopic list

```  

You can use    ``` $ rostopic pub /cmd_vel geometry_msgs/Twist <value> ``` as shown in the below image to see if your robot is responding to the velocity commands.

![irobot](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/w3_or3.png)

If you change `z: 0.0` to `z: 5.0" -r 10`, you rotate the robot. The mechanism behind is, you send `geometry_msgs/Twist` messages to the topic `cmd_vel`, which the robot listens to to get forward and angular velocity inputs. 
![irobot](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/w3_or4.png)

### iRobot Teleoperation with Keyboard     

Assuming you have installed the Teleoperation Twist Keyboard ROS package, run the ROS node for the Teleoperation Twist Keyboard

``` bash
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```    
    
Now you could use keys listed on the terminal interface to control the robot.
![irobot](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/w3_or5.png)

`teleop_twist_keyboard` is to enable keyboard control to iRobot, reading from the keyboard input and publishing it to /cmd_vel topic with the message type geometry_msgs/Twist.  For more details on this package, visit <http://wiki.ros.org/teleop_twist_keyboard>.

### Turtlesim simulator

If you do not have a physical robot like create2, then you can still test the tutorial using a simulated turtle robot using

``` bash
rosrun turtlesim turtlesim_node 
```

Note, you will have to remap the /cmd_vel topic to /turtle1/cmd_vel if you use the turtlesim.


## 5.2 Get iRobot Roomba to rotate and track an individual's face

Follow the below steps:
  
#### Step 1:  Create a node that would use HAAR cascade detector to detect an individual's face and publish those values ####

* Create a python file in your scripts folder and name it detect_face.py. 
* Save the script in your ```ros_package/scripts``` folder. Alternatively, download the ROS package called irobot_vision_tutorial supplied with this tutorial. 
* Add the following code in this script. We detail each segment of the code below.

``` bash 
#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
import sys
sys.path.append('/opt/ros/indigo/lib/python2.7/dist-packages')
```

* This code will import all the necessary packages into your script. Note, we used Python 2.7.

* The following code will initiate the ``` detect_face ``` node and begin publishing a topic ``` rotation_angle```. Since we will be using the USB camera we will be obtaining the video frames from a topic called ``` /usb_cam/image_raw/compressed ``` which we will use for detecting a person's face. The subscriber will initiate a callback function ```  image_callback ``` every time it gets a new message.

``` bash
if __name__ == '__main__':
    try:
	rospy.init_node('detect_face', anonymous=True)
	pub=rospy.Publisher('rotation_angle',Float32,queue_size=1)
	#rate = rospy.Rate(1) # 10hz
	face_cascade=cv2.CascadeClassifier('/home/arabinda/catkin_ws/src/ros_seminar/scripts/haarcascade_frontalface_default.xml')
	sub = rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage, image_callback,  queue_size = 1)
	rospy.spin()
        #talker()
    except rospy.ROSInterruptException:
	pass
```

* The ``` image_callback``` function code will utilise the ``` haarcascade_frontalface_default.xml ``` file to detect the individuals face on the compressed image it obtains as ``` ros_data ```. Subsequently, we will calculate the coordinates of the center of the bounding box and evaluate its distance from the center x-coordinate of the window frame (in my case it is 320 as the x-coordinate of window frame ranges from 0-640).
Then we will convert this distance into the degree value the robot should rotate. This degree value is published over the topic ``` rotation angle ```. 

``` bash 
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
		#rate.sleep()		
		cv2.imshow('Video',gray)
		cv2.waitKey(2)

```
![Ethcher](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/1.png)

#### Step 2: Create a new node that subscribes to ```rotation_angle```, convert's the angle value into radians and publishes it as the "angular value for the z-axis" over the /cmd_vel topic. ####

* Create a new python file in your ``` ros_package/scripts``` folder and name it rotate_robot.py.

* Import the necessary packages, initiate the python inverter and create an object of the ```Twist``` class.

``` bash
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
msg=Twist()
```

* Initiate the python script by running the ```__main__``` function and calling the ``` listener ``` function
``` bash 
if __name__ == '__main__':
    listener()
```

* The ``` listener() ``` is responsible for initializing the ``` rotate_robot ``` node and subscribing to the ``` rotation_angle ``` topic. It also passes the rotation angle data obtained from the topic to the ``` callback ``` function

``` bash 

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rotate_robot', anonymous=True)

    rospy.Subscriber('rotation_angle', Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 ``` 

* The callback function converts the angle value passed into it using ``` data ``` and converts it into radians. This value is then published ``` turtle1/cmd_vel ```

``` bash
def callback(data):
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
    pub=rospy.Publisher('turtle1/cmd_vel',Twist,queue_size=10)
    if(data.data>30):
	msg.angular.z=30*0.0174533
    elif(data.data<-30):
	msg.angular.z=-30*0.0174533
    else:
    	msg.angular.z=(data.data)*0.0174533
    pub.publish(msg)

```
![Ethcher](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/3.png)

####  Step 3: Make iRobot rotate based on the location of person's face on the camera image.

- If you have the robot installed with a camera, then run the above nodes and launch the create2 drivers ``` roslaunch ca_driver create_2.launch```  drivers and usb_cam node ``` rosrun usb_cam usb_node ``` 

- A demonstration video is available here: <https://youtu.be/ZyD-bbF6ts4>

####  Step 4: Make turtlesim rotate

- If you do not have an iRobot Roomba, launch the turtlesim by using the command ``` $ rosrun turtlesim turtlesim_node ```

- When you move face in front of the webcam the turtlesim/irobot will rotate

![Ethcher](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/blob/master/images/2.png)
 

## 5.3 Get iRobot Roomba (or turtlesim) to move towards a green colored object

This tutorial will be about detecting a green colored object and enabling any robot such as the Roomba or turtlesim to move towards the object.

This tutorial will involve the following steps:
  
#### Step 1:  Create a node to detect the ball  ####

* Create a new python file in your ``` ros_package/scripts``` folder and name it ball_tracking1.py

* Import the necessary packages

``` bash 
#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32
from collections import deque
import argparse
import imutils
import sys
```

*  The following code will initiate the script and call a ```talker()``` function that is responsible for detecting the individuals face and publishing angle values. Also when the node is stopped using ``` CTRL+C ``` the camera would be released by OpenCV and all the window frames would be destroyed, until then the talker function would be run in a loop.

``` bash
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	video_capture.release()
	cv2.destroyAllWindows()
	pass
```

* In case you do not have any green object around you, we have provided a video ``` ball_tracking_example.mp4 ``` of a person playing with a green colored ball in the code section of ``` Week 12```. Download the video and place it in your ``` ros_package/scripts``` folder.

* The talker function will run the video script or the webcam depending upon your preference. The ``` talker()``` will detect any object whose color value ranges between ``` greenLower ``` and ``` greenUpper ```. The object would be detected and a circular contour would be placed over each frame. The centroid coordinates of the circular contour would then be published using ``` coordinates ``` topic

``` bash 
def talker():
	rospy.init_node('ball_tracking1', anonymous=True)
	pub=rospy.Publisher('coordinates',Float32,queue_size=1)
	rate = rospy.Rate(20) # 10hz
        ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video", dest="/home/arabinda/catkin_ws/src/ros_seminar/scripts/ball_tracking_example.mp4",help="path")
	ap.add_argument("-b", "--buffer", type=int, default=64,help="max buffer size")
	args = vars(ap.parse_args())

	greenLower = (29, 86, 6)
	greenUpper = (64, 255, 255)
	pts = deque(maxlen=64)
	if not args.get("video", False):
		camera = cv2.VideoCapture(0)
	else:
		camera = cv2.VideoCapture('args["video"]')

	while not rospy.is_shutdown():
		(grabbed, frame) = camera.read()
		if args.get("video") and not grabbed:
			break
		frame = imutils.resize(frame, width=600)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			#print(center)
			pub.publish(int(M["m01"] / M["m00"]))
			rospy.loginfo(int(M["m01"] / M["m00"]))
			rate.sleep()
			if radius > 10:
				cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
		pts.appendleft(center)
		for i in xrange(1, len(pts)):
			if pts[i - 1] is None or pts[i] is None:
				continue
			thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
			cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

		cv2.imshow("Frame", frame)
		if cv2.waitKey(1) & 0xFF==ord('q'):
			break

	rospy.spin()
```
#### Step 2: Create a node to find the y-coordinate value and based on it publish a linear speed over /cmd_vel topic  ####

* Create a new python file in your ``` ros_package/scripts``` folder and name it y_coordinate.py

* Import the necessary packages and create an object of ``` Twist ``` class
``` bash
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
msg=Twist()
```

* Initiate the python script by running the ```__main__``` function and calling the ``` listener ``` function
``` bash 
if __name__ == '__main__':
    listener()
```

* The ``` listener() ``` is responsible for initializing the ```  y_coordinate ``` node and subscribing to the ``` coordinates ``` topic. It also passes the y_coordinate data obtained from the topic to the ``` callback ``` function

``` bash 

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rotate_robot', anonymous=True)

    rospy.Subscriber('rotation_angle', Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 ``` 

* The callback function determines the linear velocity of the irobot/turtlesim based on how far the y-coordinate is and publishes it over ``` turtle1/cmd_vel ```.

``` bash

def callback(data):
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
    pub=rospy.Publisher('turtle1/cmd_vel',Twist,queue_size=10)
    if(data.data>50):
	msg.linear.x=0.1
    else:
        msg.linear.x=0
    pub.publish(msg)


```

* Make irobot/turtlesim move linearly

- If you have the robot installed with a camera, then run the above nodes and launch the create2 and usb_cam drivers

- If you do not have an iRobot Roomba launch the turtlesim by using the command ``` $ rosrun turtlesim turtlesim_node ```

- When you place a green colored object infront of the webcam the turtlesim/irobot will move towards it

- A demonstration video is available here: <https://youtu.be/-P3i7L_g1OM>


# 6. Optional extensions - iRobot Remote control using Joystick (Gamepad)
One can also use a joystick (e.g. Logitech Wireless Gamepad F710) to control the robot remotely. We provide a brief tutorial to inspire our readers.


## Brief Introduction to Joystick
In this project, we could use Microsoft Xbox 360 Wireless Controller for Linux.
Table of index number of /joy.buttons:

Index  Button name on the actual controller    
0      A    
1      B    
2      X    
3      Y    
4      LB    
5      RB    
6      back    
7      start    
8      power    
9      Button stick left    
10     Button stick right      

Table of index number of /joy.axis:    
Index    Axis name on the actual controller    
0        Left/Right Axis stick left        
1        Up/Down Axis stick left        
2        Left/Right Axis stick right        
3        Up/Down Axis stick right        
4        RT        
5        LT        
6        cross key left/right        
7        cross key up/down        

Or we can use joystick Logitech Gamepad F710, <https://www.roscomponents.com/en/joysticks/171-logitech-gamepad-f710.html>.    


## Joystick Driver Installation   

Installing the joystick driver    

0. Start by installing the package
    ``` bash
    $ sudo apt-get install ros-indigo-joy ros-indigo-teleop-twist-joy ros-indigo-teleop-tools-msgs
    ```

1. Configuring the Joystick and make sure that the joystick is recognized by Linux working. 
    ``` bash
    $ ls /dev/input/
    $ sudo jstest /dev/input/jsX 
    ```   
   If the joystick device is not configured properly and you need to
    ``` bash
    $ sudo chmod a+rw /dev/input/jsX
    ```   


## Starting the Joy Node   
0. In this section we use ros-indigo-joy package. Assume the joystick driver has been isntalled, to get the joystick data published over ROS we need to start the joy node. First let's tell the joy node which joystick device to use- the default is js0.
    ``` bash
    $ roscore
    $ rosparam set joy_node/dev "/dev/input/jsX" 
    ```

1. Configuring the Joystick and make sure that the joystick is working. 
    ``` bash
    $ ls /dev/input/
    $ sudo jstest /dev/input/jsX 
    ```
    Now we can start the joy node 
    ``` bash
    $ rosrun joy joy_node
    ```

2. In a new terminal you can rostopic echo the joy topic to see the data from the joystick
    ``` bash
    $ rostopic echo joy 
    ```

3. Then run the teleop_twist_joy node from teleop_twist_joy package with command configurations. We recommend the following launch file which launches both joy node and teleop node with right configurations.
   ``` bash
    $ roslaunch teleop_twist_joy teleop.launch
   ```


# Summary  
After this tutorial, one should gain skills in teleoperating (control) an iRobot create2 robot. This tutorial uses ROS Indigo version and we deployed packages including teleoperation-twist-keyboard, ros-indigo-joy, and OpenCV. 
We also showed how to control a robot motion using vision based object detection and demonstrated this idea with an iRobot create2 robot. Eventually, we exhibited the use of the robot to track a ball/person using OpenCV algorithms, thus one can learn how to integrate openCV in robot control. We encourage readers to learn more about our other ROS tutorials. 

If you have any feedback please feel free to reach us through email: Arabinda Samantaray (samantar@purdue.edu) and Shaocheng Luo (scluo@purdue.edu).
