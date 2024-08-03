#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from time import sleep
import math
car_coordinate=[0,0]
prev=[0,0]
total=[[0,0,2],[1,1,2],[-1,1,2],[0,0,2]]
pres=[]
diff=0


def main():
    rospy.init_node("basic_shapes")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    rate = rospy.Rate(1)
 
    # Set our initial shape type to be a cube
    shape = Marker.LINE_STRIP

    while not rospy.is_shutdown():
        marker = Marker()

        # Set the frame ID and timestamp
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # Set the namespace and ID for this marker
        marker.ns = "basic_shapes"
        marker.id = 0

        # Set the marker type
        marker.type = shape

        # Set the marker action
        marker.action = Marker.ADD

        # Set the pose of the marker
        
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.0
        marker.scale.z = 0.0

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Define the points for the line strip
        global total
        print("tatal=",total)
        
        for i in total:
            p=Point()
            p.x=i[0]
            p.y=i[1]
            p.z=i[2]
            marker.points.append(p)
        
        '''
        global car_coordinate 
        p1 = Point()
        p1.x=car_coordinate[0]
        p1.y=car_coordinate[1]
        p1.z=0

        marker.points.append(p1)

        
        '''
       
        #marker.points=total
       
        marker_pub.publish(marker)
       

        # Cycle between different shapes

        rate.sleep()

if __name__ == "__main__":
    
    main()
   
    
