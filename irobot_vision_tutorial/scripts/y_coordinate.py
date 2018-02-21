#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
msg=Twist()
def callback(data):
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    if(data.data>300):
	msg.linear.x=0.8
    elif((data.data>100) and (data.data<300)):
	msg.linear.x=0.5
    elif((data.data > 0) and (data.data<100)):
    	msg.linear.x=0.1
    else:
	msg.angular.z=-0.174533
    pub.publish(msg)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('y_coordinate', anonymous=True)

    rospy.Subscriber('coordinates', Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
