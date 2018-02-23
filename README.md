ROS tutorial by Purdue SMART lab: iRobot Create2 teleoperation and Computer Vision based object detection for mobile robot control.

# Objectives
This ROS tutorial provides an overview of teleoperating (control) an iRobot create2 (roomba) mobile robot. Then, we look at ways to make use of the robot to track a ball/person using openCV algorithms. For those who do not have a creat2 robot, we also show a case of controlling a turtlebot in simulation (instead of iRobot control) in this tutorial. 
Additionally, this tutorial will help you learn some basics of computer vision and robot control methods in ROS.

## Demonstration video:
Please watch the videos below to get an idea of what is expected to be achieved by this tutorial.

<https://www.youtube.com/watch?v=ZyD-bbF6ts4>

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/ZyD-bbF6ts4/0.jpg)](https://www.youtube.com/watch?v=ZyD-bbF6ts4)

<https://www.youtube.com/watch?v=ZyD-bbF6ts4>

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/-P3i7L_g1OM/0.jpg)](https://www.youtube.com/watch?v=-P3i7L_g1OM)



# Credits
This tutorial is prepared by Arabinda Samantaray (samantar@purdue.edu) and Shaocheng Luo (scluo@purdue.edu).

The git repository is maintained by Ramviyas Parasuraman (ramviyas@purdue.edu)

We acknowledge the following sources which were used to prepare this tutorial:

- http://wiki.ros.org/ROS/Tutorials

- http://milq.github.io/install-opencv-ubuntu-debian/

- https://docs.opencv.org/3.3.0/d7/d8b/tutorial_py_face_detection.html

- https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

- ROS create_autonomy package summary <http://wiki.ros.org/create_autonomy>    

- ROS driver for iRobot Create 1 and 2 <https://github.com/autonomylab/create_autonomy>    

- ROS joy <http://wiki.ros.org/joy>

# Prerequisites
A basic background of using Linux-based OS, ROS and OpenCV will be required to understand the following tutorial. If you need some help, please visit:

- http://wiki.ros.org/ROS/Tutorials

- http://files.ubuntu-manual.org/manuals/getting-started-with-ubuntu/14.04e2/en_US/screen/Getting%20Started%20with%20Ubuntu%2014.04%20-%20Second%20edition.pdf

- https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_tutorials.html

- http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

This tutorial was tested successfully in Ubuntu 14.04, ROS-indigo with OpenCV 3.0.0.

# Tutorial

* Section 1: [Installation of relevant dependencies/packages](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/Installation)

* Section 2: [iRobot Create2 robot teleoperation control with Keyboard](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/iRobot-control-with-a-keyboard)

* Section 3: [Rotate the robot to follow a person's face (using HAAR cascade filter and openCV)](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/Robot-control-with-face-tracking)

* Section 4: [Move the robot to follow a green color object (using openCV)](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/Robot-control-with-colored-object-detection)

* Section 5: [Optional extension - iRobot Remote control with a Gamepad/Joystick](https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/Robot-control-with-Joystick)


# Summary  

After this tutorial, one should gain skills in teleoperating (control) an iRobot create2 robot. This tutorial uses ROS Indigo version and we deployed packages including teleoperation-twist-keyboard, ros-indigo-joy, and OpenCV. 
We also showed how to control a robot motion using vision-based object detection and demonstrated this idea with an iRobot create2 robot. Eventually, we exhibited the use of the robot to track a ball/person using OpenCV algorithms, thus one can learn how to integrate openCV in robot control. We encourage readers to learn more about our other ROS tutorials. 

If you have any feedback please feel free to reach the authors through email: Arabinda Samantaray (samantar@purdue.edu) and Shaocheng Luo (scluo@purdue.edu).
