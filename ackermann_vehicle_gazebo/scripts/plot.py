#!/usr/bin/env python
import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist,Pose
import numpy as np                    #import numpy for trignometric function, arrays... etc
import sys                            #import sys for extracting input from termminal (input from user)
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl
from IPython.display import display


import matplotlib.rcsetup as rcsetup
print(rcsetup.all_backends)



#i=3
x=[]
y=[]
th=[]
#while i<6:
 #x.append(i)
 #y.append(0)
 #th.append(0)

 #i=i+0.5
#i=0
#xs=		x[-1]
#j=6

x_count=0
y_count=0
x.append(x_count)
y.append(y_count)
th.append(0)
x_count= x_count+0.2
j=1
while x_count<10:
 x.append(x_count)
 y.append(1-(1/(1+(0.3*x_count-0.01)**(9.0))))
 th.append(np.arctan2(y[j]-y[j-1],x[j]-x[j-1]))	
 x_count=x_count+0.2
 j=j+1


print(x[23])
print(y[24])

