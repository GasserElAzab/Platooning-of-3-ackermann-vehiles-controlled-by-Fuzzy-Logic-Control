#!/usr/bin/env python
# A basic python code to implement the kinematics model for the differential mobile robot
# and reflect its behavior on graphs.

#########################################################################################################
#Import the required libraries:
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



#########################################################################################################

#########################################################################################################
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

#x= [0, 0.2, 0.4, 0.6000000000000001, 0.8, 1.0, 1.2, 1.4, 1.5999999999999999, 1.7999999999999998, 1.9999999999999998, 2.1999999999999997, 2.4, 2.5999999999999996, 2.8, 3.0, 3.1999999999999997, 3.3999999999999995, 3.5999999999999996, 3.8, 4.0, 4.2, 4.4, 4.6000000000000005, 4.800000000000001, 5.000000000000001, 5.200000000000001, 5.400000000000001, 5.600000000000001, 5.800000000000002, 6.000000000000002, 6.198669330795063, 6.389418342308653, 6.564642473395037, 6.717356090899525, 6.8414709848078985, 6.932039085967228, 6.985449729988462, 6.999573603041507, 6.973847630878197, 6.909297426825684, 6.808496403819592, 6.675463180551152, 6.515501371821466, 6.334988150155906, 6.141120008059868, 6.141120008059868, 5.941120008059868, 5.741120008059868, 5.541120008059868, 5.3411200080598675, 5.141120008059867, 4.941120008059867, 4.741120008059867, 4.541120008059867, 4.341120008059867, 4.1411200080598665, 3.9411200080598663, 3.741120008059866, 3.541120008059866, 3.3411200080598658, 3.1411200080598656, 2.9411200080598654, 2.741120008059865, 2.541120008059865, 2.341120008059865, 2.1411200080598647, 1.9411200080598647, 1.7411200080598648, 1.5411200080598648, 1.3411200080598649, 1.141120008059865, 0.941120008059865, 0.741120008059865, 0.541120008059865, 0.34112000805986503, 0.14112000805986502]

#y= [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.19866933079506122, 0.3894183423086505, 0.5646424733950355, 0.7173560908995228, 0.8414709848078965, 0.9320390859672263, 0.9854497299884601, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 0.9995736030415052, 1.0995736030415053, 1.1995736030415052, 1.2995736030415053, 1.3995736030415054, 1.4995736030415052, 1.599573603041505, 1.6995736030415052, 1.7995736030415053, 1.8995736030415051, 1.999573603041505, 2.099573603041505, 2.199573603041505, 2.2995736030415053, 2.3995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054, 2.4995736030415054]

#th= [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12.080374914413625, 6.676624384424234, 4.9588901507256296, 4.182023458165908, 3.8028643384899876, 3.6479156842136504, 3.653154382661566, 3.801620999631593, 4.001706315401677, 4.20179163117176, 4.401876946941845, 4.601962262711929, 4.8020475784820125, 5.002132894252097, 5.20221821002218, 5.402303525792265, 5.602388841562349, 5.8024741573324325, 6.002559473102517, 5.63733915915139, 5.326407922038594, 5.051381820953606, 4.799573295967855, 4.562277550719556, 4.333679346037169, 4.110118983660086, 3.88957339183646, 3.6712700259216122, 3.4553853963245524, 3.242799582713652, 3.0348896583049187, 2.8333519584690876, 2.6400474409812595, 2.4568670434778532, 2.4568670434778532, 2.37685339644756, 2.2968397494172677, 2.2168261023869746, 2.136812455356682, 2.056798808326389, 1.9767851612960965, 1.8967715142658037, 1.816757867235511, 1.7367442202052181, 1.6567305731749253, 1.5767169261446325, 1.4967032791143398, 1.416689632084047, 1.3366759850537542, 1.2566623380234614, 1.1766486909931686, 1.0966350439628758, 1.016621396932583, 0.9366077499022902, 0.8565941028719974, 0.7765804558417048, 0.6965668088114121, 0.6165531617811194, 0.5365395147508267, 0.456525867720534, 0.3765122206902413, 0.29649857365994864, 0.21648492662965593, 0.13647127959936323, 0.056457632569070514]

#Gasser traj start
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
# Gasser traj end
#########################################################################################################
#########################################################################################################
#Initialize ROS Node
rospy.init_node('Point_to_Point_Control', anonymous=True) #Identify ROS Node
#######################################################################
#######################################################################
#######################################################################
def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]
#######################################################################
flag_initial_Pos = 0	#Initialize flag by zero
xcordinit = 0 
ycordinit = 0
thetayawinit = 0
xcord = 0
ycord = 0
yaw = 0

#######################################################################
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
#######################################################################



#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)#Identify the publisher "pub1" to publish on topic "/turtle1/cmd_vel" to send message of type "Twist"
rate = rospy.Rate(10) # rate of publishing msg 10hz
zizo = rospy.Publisher("/kalboz", Float64, queue_size=10)
#######################################################################
#######################################################################

#######################################################################
#ROS Subscriber Code for Position
flag_cont = 0	#Initialize flag by zero
pos_msg = Pose()	#Identify msg variable of data type Pose
position = np.zeros((1,6))
Velocity_msg = Twist()
velocity = np.zeros((1,6))


#pos_msg_0 = Pose()	#Identify msg variable of data type Pose
#pos_msg = Pose()	#Identify msg variable of data type Pose
#######################################################################



def callback(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub2		#Identify a subscriber as global variable
  global flag_cont
  global position 
  global Velocity_msg
  global velocity

  msg = data
  pos_msg.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  Velocity_msg.linear.x = round(msg.twist.twist.linear.x, 4)
  Velocity_msg.linear.y = round(msg.twist.twist.linear.y, 4)
  Velocity_msg.linear.z = round(msg.twist.twist.linear.z, 4)
  Velocity_msg.angular.x = round(msg.twist.twist.angular.x, 4)
  Velocity_msg.angular.y = round(msg.twist.twist.angular.y, 4)
  Velocity_msg.angular.z = round(msg.twist.twist.angular.z, 4)
  velocity = [Velocity_msg.linear.x,Velocity_msg.linear.y,Velocity_msg.linear.z,Velocity_msg.angular.x,Velocity_msg.angular.y,Velocity_msg.angular.z]
  flag_cont = 1

sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
#######################################################################

#######################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	#Identify msg variable of data type Pose
position_0 = np.zeros((1,6))
flag_initial = 0
Velocity_msg_0 = Twist()
velocity_0 = np.zeros((1,6))


#######################################################################
#######################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 

def callback_Init(data):
  global pos_msg_0		#Identify msg variable created as global variable
  global sub1			#Identify a subscriber as global variable
  global flag_initial 	#Identify flag created as global variable
  global position_0 
  global Velocity_msg_0
  global velocity_0

  msg = data
  pos_msg_0.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg_0.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg_0.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg_0.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position_0 = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  Velocity_msg_0.linear.x = round(msg.twist.twist.linear.x, 4)
  Velocity_msg_0.linear.y = round(msg.twist.twist.linear.y, 4)
  Velocity_msg_0.linear.z = round(msg.twist.twist.linear.z, 4)
  Velocity_msg_0.angular.x = round(msg.twist.twist.angular.x, 4)
  Velocity_msg_0.angular.y = round(msg.twist.twist.angular.y, 4)
  Velocity_msg_0.angular.z = round(msg.twist.twist.angular.z, 4)
  velocity_0 = [Velocity_msg_0.linear.x,Velocity_msg_0.linear.y,Velocity_msg_0.linear.z,Velocity_msg_0.angular.x,Velocity_msg_0.angular.y,Velocity_msg_0.angular.z]
  flag_initial = 1
  sub1.unregister()				#Unsubsribe from this topic

sub1 = rospy.Subscriber('/odom', Odometry, callback_Init) #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
#######################################################################
#######################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
  pass
#######################################################################






#########################################################################################################
#Define the initial pose of the vehicle: Can get it from /turtle1/pose
x0 = position_0[0]
y0 = position_0[1]
theta0 = position_0[3]
#######################################################################
#######################################################################
#Initialize the parameters for coordinate transformation
rho = 0 #Initialization of variable rho
beta = 0 #Initialization of variable beta
alpha = 0 #Initialization of variable alpha
#######################################################################
#######################################################################
#Initialize the control gains
Krho = 0.38
Kalpha = 1.2
Kbeta = -0.15
#######################################################################
#######################################################################
#Initialize controller output
linear_v = 0 #Initialize linear velocity 
angular_v = 0 #Initialize angular velocity
#########################################################################################################

#########################################################################################################
#Coordinate transformation function to transform to polar coordinates

def transformation(xcord, ycord, yaw):
    global rho		#Identify rho variable as global variable
    global beta		#Identify beta variable as global variable
    global alpha	#Identify alpha variable as global variable
    global x_des
    global y_des
    global theta_des



    xco = xcord
    yco = ycord
    yawco = yaw

    x_delta = x_des - xco		#Calculate the change in X direction
    y_delta = y_des - yco	#Calculate the change in Y direction

    #Calculate distance rho representing relative distance between the desired and the current position
    rho = np.sqrt((np.square(x_delta))+ (np.square(y_delta)))
    #Calculate angle gamma representing the angle between the global X-direction of the vehicle and rho
    

    #Calculate angle alpha between the local X-direction of the vehicle and rho 
    alpha = theta_des - yawco
    #Calculate angle beta between rho and desired global X-direction of the vehicle
    beta = - alpha - yawco + (theta_des*(np.pi/180))
#########################################################################################################
#########################################################################################################

def fuzzy_control (y_actual, y_desired, theta_actual, theta_desired, omega_actual, omega_desired ):

# Generate universe variables
#   * Quality and service on subjective ranges [0, 10]
#   * Tip has a range of [0, 25] in units of percentage points
    LateralDisplacement = ctrl.Antecedent(np.arange(-3, 3, 0.5), 'LateralDisplacement')
    SteeringAngle1  = ctrl.Consequent(np.arange(-0.4, 0.4, 0.01),'SteeringAngle1')


# Generate fuzzy membership functions
    LateralDisplacement['R'] = fuzz.trapmf(LateralDisplacement.universe, [-100, -3, -1.5, 0])
    LateralDisplacement['C'] = fuzz.trimf(LateralDisplacement.universe, [-1.5, 0, 2])
    LateralDisplacement['L'] = fuzz.trapmf(LateralDisplacement.universe,[0,2,3,100])

#LateralDisplacement.view()


    SteeringAngle1['R'] = fuzz.trimf(SteeringAngle1.universe, [-0.4, -0.4, 0])
    SteeringAngle1['C'] = fuzz.trimf(SteeringAngle1.universe, [-0.005, 0, 0.005])
    SteeringAngle1['L'] = fuzz.trimf(SteeringAngle1.universe,[0,0.4,0.4])


#SteeringAngle1.view()


    rule_1 = ctrl.Rule(LateralDisplacement['C'] , SteeringAngle1['C'])
    rule_2 = ctrl.Rule(LateralDisplacement['R'] , SteeringAngle1['R'])
    rule_3 = ctrl.Rule(LateralDisplacement['L'] , SteeringAngle1['L'])
    SteeringAngle1_cntrl = ctrl.ControlSystem([rule_1, rule_2, rule_3])
    SteeringAngle1_out = ctrl.ControlSystemSimulation(SteeringAngle1_cntrl)



    # >>>>>>>>>>>>>>>>>>>>>>>> goz2 wa7oshh
    YawAngleError = ctrl.Antecedent(np.arange(-0.5, 0.5, 0.01), 'AngleError')
    YawRateError = ctrl.Antecedent(np.arange(-1, 1, 0.01),'RateError')
    SteeringAngle  = ctrl.Consequent(np.arange(-0.5, 0.5, 0.01),'SteeringAngle')


    # Generate fuzzy membership functions
    YawAngleError['NB'] = fuzz.trapmf(YawAngleError.universe, [-1, -0.5, -0.4, -0.2])
    YawAngleError['NM'] = fuzz.trimf(YawAngleError.universe, [-0.35, -0.2, -0.05])
    YawAngleError['NS'] = fuzz.trimf(YawAngleError.universe,[-0.2,-0.1,0])
    YawAngleError['ZE'] = fuzz.trimf(YawAngleError.universe,[-0.05, 0, 0.05])
    YawAngleError['PS'] = fuzz.trimf(YawAngleError.universe,[0, 0.1, 0.2])
    YawAngleError['PM'] = fuzz.trimf(YawAngleError.universe,[0.05, 0.2, 0.35])
    YawAngleError['PB'] = fuzz.trapmf(YawAngleError.universe,[0.2, 0.4,0.5, 1])

    #YawAngleError.view()

    YawRateError['NB'] = fuzz.trapmf(YawRateError.universe, [-2, -1, -0.8, -0.2])
    YawRateError['NM'] = fuzz.trimf(YawRateError.universe, [-0.8, -0.4, 0])
    YawRateError['ZE'] = fuzz.trimf(YawRateError.universe,[-0.2, 0, 0.2])
    YawRateError['PM'] = fuzz.trimf(YawRateError.universe,[0, 0.4, 0.8])
    YawRateError['PB'] = fuzz.trapmf(YawRateError.universe,[0.2, 0.8, 1, 2])

    #YawRateError.view()

    SteeringAngle['NB'] = fuzz.trapmf(SteeringAngle.universe, [-1, -0.5, -0.4, -0.2])
    SteeringAngle['NM'] = fuzz.trimf(SteeringAngle.universe, [-0.35, -0.2, -0.05])
    SteeringAngle['NS'] = fuzz.trimf(SteeringAngle.universe,[-0.2,-0.1,0])
    SteeringAngle['ZE'] = fuzz.trimf(SteeringAngle.universe,[-0.05, 0, 0.05])
    SteeringAngle['PS'] = fuzz.trimf(SteeringAngle.universe,[0, 0.1, 0.2])
    SteeringAngle['PM'] = fuzz.trimf(SteeringAngle.universe,[0.05, 0.2, 0.35])
    SteeringAngle['PB'] = fuzz.trapmf(SteeringAngle.universe,[0.2, 0.4,0.5, 1])

    #SteeringAngle.view()


    rule1 = ctrl.Rule(YawAngleError['NB'] & YawRateError['NB'], SteeringAngle['NB'])
    rule2 = ctrl.Rule(YawAngleError['NB'] & YawRateError['NM'], SteeringAngle['NB'])
    rule3 = ctrl.Rule(YawAngleError['NB'] & YawRateError['ZE'], SteeringAngle['NM'])
    rule4 = ctrl.Rule(YawAngleError['NB'] & YawRateError['PM'], SteeringAngle['NM'])
    rule5 = ctrl.Rule(YawAngleError['NB'] & YawRateError['PB'], SteeringAngle['NS'])
    rule6 = ctrl.Rule(YawAngleError['NM'] & YawRateError['NB'], SteeringAngle['NB'])
    rule7 = ctrl.Rule(YawAngleError['NM'] & YawRateError['NM'], SteeringAngle['NM'])
    rule8 = ctrl.Rule(YawAngleError['NM'] & YawRateError['ZE'], SteeringAngle['NM'])
    rule9 = ctrl.Rule(YawAngleError['NM'] & YawRateError['PM'], SteeringAngle['NS'])
    rule10 = ctrl.Rule(YawAngleError['NM'] & YawRateError['PB'], SteeringAngle['NS'])
    rule11 = ctrl.Rule(YawAngleError['NS'] & YawRateError['NB'], SteeringAngle['NM'])
    rule12 = ctrl.Rule(YawAngleError['NS'] & YawRateError['NM'], SteeringAngle['NM'])
    rule13 = ctrl.Rule(YawAngleError['NS'] & YawRateError['ZE'], SteeringAngle['NM'])
    rule14 = ctrl.Rule(YawAngleError['NS'] & YawRateError['PM'], SteeringAngle['NS'])
    rule15 = ctrl.Rule(YawAngleError['NS'] & YawRateError['PB'], SteeringAngle['NS'])
    rule16 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['NM'], SteeringAngle['ZE'])
    rule17 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['ZE'], SteeringAngle['ZE'])
    rule18 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['PB'], SteeringAngle['ZE'])
    rule19 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['NB'], SteeringAngle['ZE'])
    rule20 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['PM'], SteeringAngle['ZE'])
    rule21 = ctrl.Rule(YawAngleError['PS'] & YawRateError['NB'], SteeringAngle['PS'])
    rule22 = ctrl.Rule(YawAngleError['PS'] & YawRateError['NM'], SteeringAngle['PS'])
    rule23 = ctrl.Rule(YawAngleError['PS'] & YawRateError['ZE'], SteeringAngle['PM'])
    rule24 = ctrl.Rule(YawAngleError['PS'] & YawRateError['PM'], SteeringAngle['PM'])
    rule25 = ctrl.Rule(YawAngleError['PS'] & YawRateError['PB'], SteeringAngle['PM'])
    rule26 = ctrl.Rule(YawAngleError['PM'] & YawRateError['NB'], SteeringAngle['PS'])
    rule27 = ctrl.Rule(YawAngleError['PM'] & YawRateError['NM'], SteeringAngle['PS'])
    rule28 = ctrl.Rule(YawAngleError['PM'] & YawRateError['ZE'], SteeringAngle['PM'])
    rule29 = ctrl.Rule(YawAngleError['PM'] & YawRateError['PM'], SteeringAngle['PM'])
    rule30 = ctrl.Rule(YawAngleError['PM'] & YawRateError['PB'], SteeringAngle['PB'])
    rule31 = ctrl.Rule(YawAngleError['PB'] & YawRateError['NB'], SteeringAngle['PS'])
    rule32 = ctrl.Rule(YawAngleError['PB'] & YawRateError['NM'], SteeringAngle['PM'])
    rule33 = ctrl.Rule(YawAngleError['PB'] & YawRateError['ZE'], SteeringAngle['PM'])
    rule34 = ctrl.Rule(YawAngleError['PB'] & YawRateError['PM'], SteeringAngle['PB'])
    rule35 = ctrl.Rule(YawAngleError['PB'] & YawRateError['PB'], SteeringAngle['PB'])

    SteeringAngle2_cntrl = ctrl.ControlSystem([rule1, rule2, rule3,rule4,rule5,rule6,rule7,rule8,rule9,rule10,rule11,rule12,rule13,rule14,rule15,rule16,rule17,rule18,rule19,rule20,rule21,rule22,rule23,rule24,rule25,rule26,rule27,rule28,rule29,rule30,rule31,rule32,rule33,rule34,rule35])

    SteeringAngle2_out = ctrl.ControlSystemSimulation(SteeringAngle2_cntrl)


# FINAL STEP AFTER 2 FUZZY 
 
 
#>>>>> wahsh fuzzy
    thetaError =  theta_desired - theta_actual
    OmegaError =  omega_desired - omega_actual
    YError = y_desired- y_actual 
    SteeringAngle2_out.input['AngleError'] = thetaError
    SteeringAngle2_out.input['RateError'] = OmegaError

    SteeringAngle2_out.compute()

    St_wahsh= SteeringAngle2_out.output['SteeringAngle']
#>>>>> gasser fuzzy 

    SteeringAngle1_out.input['LateralDisplacement'] = YError

    SteeringAngle1_out.compute()

    St_gasser= SteeringAngle1_out.output['SteeringAngle1']


#final result
    fuzzy_out= St_gasser* 0.6 + St_wahsh *0.4

    return fuzzy_out




#########################################################################################################



p=0
t=[]
t.append(0)
xplot_actual=[]
xplot_actual.append(0)
yplot_actual=[]
yplot_actual.append(0)
xplot_desired=[]
xplot_desired.append(0)
yplot_desired=[]
yplot_desired.append(0)
thetaplot_desired=[]
thetaplot_desired.append(0)
thetaplot_actual=[]
thetaplot_actual.append(0)
omegazz=[]
omegazz.append(0)
printed=False



while not rospy.is_shutdown()  and p<len(x)-1:
    #rospy.loginfo(x0)
    #Call the transformation function
    if rho <0.7  and p < len(x)-1:
	p=p+1
    x_des=x[p]
    y_des=y[p]
    theta_des=th[p]

    transformation(position[0], position[1], position[3])
    #Call the control function
    if np.absolute(alpha) < np.pi/2 and alpha != 0 or x_des<position[0]: #Condition handles if desired position is infornt or behind the vehicle
	linear_v = Krho * rho
    else:
        linear_v = -Krho * rho


    x_actual= position[0]
    y_actual= position[1]
    theta_actual= position[3]
    omega_actual= velocity[5]
    omega_desired=1.5
    angular_v = fuzzy_control (y_actual, y_des, theta_actual, theta_des, omega_actual, omega_desired )


    print("The point trajectory at " + str(p))
    print("The fuzzy output is >>>>>>" + str(angular_v))
    print("rho equal to  >>>>" + str(rho) + " <<<<<theta desired >>>>> " + str (theta_des) + " <<<<<<<<<< theta actual >>>>>>> " + str(theta_actual))
    #Calculate the linear and angular velocities
    v = round(linear_v,2) 	#Linear Velocity
    w = round(angular_v,2)	#Angular Velocity

    ackermann_cmd_msg = AckermannDriveStamped()
    ackermann_cmd_msg.drive.speed = v
    ackermann_cmd_msg.drive.steering_angle = w
    t.append(t[len(t)-1]+1)
    xplot_actual.append(x_actual)
    yplot_actual.append(y_actual)
    thetaplot_actual.append(theta_actual)
    xplot_desired.append(x_des)
    yplot_desired.append(y_des)
    thetaplot_desired.append(theta_des)
    omegazz.append(w)
    if printed == False and p==len(x)-2:
	printed =True
	print("omega")
	print(omegazz)
	#print("yplot actual")
	#print(yplot_actual)
	#print("xplot desired")
	#print(xplot_actual)
	#print("yplot dessired")
	#print(yplot_desired)
	#print("theta_actual")
	#print(thetaplot_actual)
	#print(thetaplot_desired)
	#print("time")
	#print(t)
	
    
	
	

    #ROS Code Publisher
 
    pub1.publish(ackermann_cmd_msg)	#Publish msg
    zizo.publish(w)
    rate.sleep()		#Sleep with rate  





	
#########################################################################################################

