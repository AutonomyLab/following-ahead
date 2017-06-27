#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



rospy.init_node('joystick', anonymous=True)
r = rospy.Rate(10) #10hz
msg = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def callbackJoy(data):
    global msg
    print (data.axes[1])
    print (data.axes[0])
    msg.linear.x = data.axes[3]
    msg.linear.z = data.axes[2]
    pub.publish(msg)
    
rospy.Subscriber("joy", Joy, callbackJoy)
def talker():    
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass


