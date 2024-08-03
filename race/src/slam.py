#!/usr/bin/env python3
import rospy
import math
import numpy as np
from race.msg import perception
from race.msg import slam
from std_msgs.msg import Float32MultiArray
from race.msg import final_coordinates
from geometry_msgs.msg import PoseStamped

left_cone_coordinates=[]
right_cone_coordinates=[]

b=True


car_coordinate=[0,0]
min_distance=10000
min_left_coordinate=[-1,-1]
min_right_coordinate=[-1,-1]
pub = rospy.Publisher('/slam_to_distfinder', slam, queue_size=10)

def distance(x,y):
	d=math.sqrt(((x[0]-y[0])**2)+((x[1]-y[1])**2))
	return d
def c(data):
    global car_coordinate 
    car_coordinate[0]=data.x
    car_coordinate[1]=data.y
def call(data):
	global car_coordinate,b
	if(b):
		car_coordinate[0]=data.pose.position.x
		car_coordinate[1]=data.pose.position.y
		b=False

def callback(data):
	global min_distance
	global min_left_coordinate
	global min_right_coordinate
	

	l=int(len(data.oned_list)/4)
	d=(np.array(data.oned_list).reshape((l, 4)))
	cones=d.tolist()
	#print(cones)

	if(len(left_cone_coordinates)==0 or len(right_cone_coordinates)==0):
		reference_x=car_coordinate[0]
		reference_y=car_coordinate[1]
	else:
		reference_x=(left_cone_coordinates[-1][0]+right_cone_coordinates[-1][0])/2
		reference_y=(left_cone_coordinates[-1][1]+right_cone_coordinates[-1][1])/2
	reference=[reference_x,reference_y]
	print("reference=",reference)
	print("before=",cones)
	if(len(left_cone_coordinates)!=0 and len(right_cone_coordinates)!=0):
		for j in range(0,len(left_cone_coordinates)):
			for i in cones:
				if(distance(left_cone_coordinates[j],i)<1 or distance(right_cone_coordinates[j],i)<1):
					cones.remove(i)
	print("after",cones)
	
	for i in range(0,len(cones)):
		for j in range(i+1,len(cones)):
			mid_x=(cones[i][0]+cones[j][0])/2
			mid_y=(cones[i][1]+cones[j][1])/2
			mid=[mid_x,mid_y]
			minimum=distance(reference,mid)
			print("first cone=",cones[i])
			print("second cone=",cones[j])
			print("distance between them=",minimum)
			if(minimum<min_distance):
				min_distance=minimum
				#the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
				#be the position of right cone and cones[j] will ve the position of left cone
				min_left_coordinate=cones[j]
				min_right_coordinate=cones[i]
				print("min left coordinate=",min_left_coordinate)
				print("min right coordinate=",min_right_coordinate)
	#print("min=",minimum)
	#print("min left final=",min_left_coordinate)
	#print("min right final=",min_right_coordinate)
	
	if(len(left_cone_coordinates)==0 and distance(reference,car_coordinate)<0.4):
		left_cone_coordinates.append(min_left_coordinate)
		right_cone_coordinates.append(min_right_coordinate)
		
		message=slam()
		message.leftcone=min_left_coordinate
		message.rightcone=min_right_coordinate
		pub.publish(message)
		
		min_distance=10000
		min_left_coordinate=[-1,-1]
		min_right_coordinate=[-1,-1]
	else:
		A1=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
		B1=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
		C1=(left_cone_coordinates[-1][0]*right_cone_coordinates[-1][1])-(right_cone_coordinates[-1][0]*left_cone_coordinates[-1][1])
		d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))
		if(d<0.4):
			left_cone_coordinates.append(min_left_coordinate)
			right_cone_coordinates.append(min_right_coordinate)
			message=slam()
			message.leftcone=min_left_coordinate
			message.rightcone=min_right_coordinate
			pub.publish(message)
			min_distance=10000
			min_left_coordinate=[-1,-1]
			min_right_coordinate=[-1,-1]
	print("left=",left_cone_coordinates)
	print("right=",right_cone_coordinates)
	print("--------------------------------------------------------------------------------------------------------------------")

if __name__ == '__main__':
	print("SLAM")
	rospy.init_node('slam',anonymous = True)
	rospy.Subscriber("/perception_to_slam",perception,callback)
	rospy.Subscriber("/gt_pose",PoseStamped, call)
	rospy.Subscriber("/final_coordinates",final_coordinates, c)
	rospy.spin()