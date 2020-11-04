#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math


class OdometryModifier:

  def __init__(self):
    self.sub = rospy.Subscriber("/robot1/odom", Odometry, self.callback)
  def callback(self, data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y 
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print "x= ",x
    print "y= ",y
    print "angle= ",yaw*180/math.pi
 

if __name__ == '__main__':
    rospy.init_node('move_turtlebot', anonymous=True)
    try:
        odom = OdometryModifier()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
