#!/usr/bin/env python3
import math
import rospy
from race.msg import pid_input
from race.msg import ekf
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global kp 
kp = 1 #1
global kd
kd = 3 #1.0
global ki
ki = 0.0
prev_error = 0.0
prev_c=0
diff=0
car_coordinate=[0,0]
original=[100,0]
vel_input = 0.5
command_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)
pub = rospy.Publisher('/controltoekf', ekf, queue_size=1)
prevsteer=0
steer=0


def call(data):
	global steer
	steer=data.drive.steering_angle


def control(data: pid_input):
	global prev_error, steer
	global vel_input, prevsteer
	global kp
	global kd
	global angle
	global prev_c, original, diff, car_coordinate
	angle=0.0
	angle = kp*data.pid_error + kd*((prev_error - data.pid_error ))
	#if(diff>=0.0009):
		
	command = AckermannDrive()
	an=command.steering_angle-angle
	'''
	if(an>0.34):
		an=0.087
	if(an<-0.34):
		an=-0.087
	'''
	print("correction angle=",an,math.degrees(an))
	#print("steering angle",steer-prevsteer,math.degrees(steer-prevsteer))
	#print("difference=",an-(steer-prevsteer))
	#prevsteer=steer
		#print("difference control=",an-prev_c)
	prev_c=an
	print("--------------------------------------------------------------------------------------------------------------------------------------------------------------------")
	command.steering_angle = an
	command.speed = vel_input
	command.steering_angle_velocity = 10.0
	command.acceleration = 0.0
			
			
	new= AckermannDriveStamped()
	new.drive=command
	command_pub.publish(new)
	message=ekf()
	controls=[vel_input,an]
	message.controls=controls
	pub.publish(message)
	prev_error = data.pid_error
	#original[0]=car_coordinate[0]
	#original[1]=car_coordinate[1]
	
	'''
	if(diff<0.0009):
		
		command = AckermannDrive()
		an=command.steering_angle
		command.steering_angle = an
		command.speed = vel_input
		command.steering_angle_velocity = 10.0
		command.acceleration = 0.0	
		new= AckermannDriveStamped()
		new.drive=command
		command_pub.publish(new)
		message=ekf()
		controls=[vel_input,an]
		message.controls=controls
		pub.publish(message)
		print("a")
	'''
def callback(data):
	global car_coordinate, original, diff
	car_coordinate[0]=data.pose.position.x
	car_coordinate[1]=data.pose.position.y
	diff=math.sqrt(((abs(car_coordinate[1]-original[1]))**2) + ((abs(car_coordinate[0]-original[0]))**2))
	#print("dist changed=",diff)
	
if __name__ == '__main__':
	rospy.init_node('pid_controller', anonymous=True)
	print("PID Control Node is Listening to error")
	rospy.Subscriber("/gt_pose",PoseStamped, callback)
	rospy.Subscriber("/rand_drive",AckermannDriveStamped, call)
	rospy.Subscriber("/err", pid_input, control)
	rospy.spin() 
