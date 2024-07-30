#!/usr/bin/env python3
import rospy
import math
import numpy as np
from rosgraph_msgs.msg import Clock
from race.msg import perception
from race.msg import ekf
from race.msg import perception
from datetime import datetime
from race.msg import slam
from geometry_msgs.msg import PoseStamped
from numpy.linalg import LinAlgError 
from race.msg import final_coordinates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

car_coordinate=[0,0]
prev_coordinate=[0,0]
bt=True
prev_m=0
prev_s=0
prev_ns=0
delta_t=0
sum=0
i=1


def real(data):
    global roll, pitch, yaw
    orientation_q = data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(math.degrees(yaw))





if __name__ == '__main__':
    print("Hokuyo LIDAR node started")
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber("/gt_pose",PoseStamped, real)
    # rospy.Subscriber("/perception_to_slam",perception,real)
    
    rospy.spin()