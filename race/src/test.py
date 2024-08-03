#!/usr/bin/env python3
import rospy
from race.msg import slam
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Initialize global variables for cone coordinates
car_coordinate = [0, 0]
lx, ly, rx, ry = 0, 0, 0, 0
total=[]
def call(data):
    global lx, ly, rx, ry
    if data.leftcone and data.rightcone:
        total.append(data.leftcone)
        total.append(data.rightcone)

def main():
    rospy.init_node("cones")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    rospy.Subscriber("/slam_to_distfinder", slam, call)
    
    rate = rospy.Rate(1)
    shape = Marker.LINE_LIST

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cones"
        marker.id = 0
        marker.type = shape
        marker.action = Marker.ADD
        marker.pose.position.x = lx
        marker.pose.position.y = ly
        marker.pose.position.z = 0
        marker.scale.x = 0.2
        marker.scale.y = 0.0
        marker.scale.z = 1.0
        marker.color.r = 10.0
        marker.color.g = 5.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        global total
        print("tatal=",total)
        
        for i in total:
            p=Point()
            p.x=i[0]
            p.y=i[1]
            p.z=0
            marker.points.append(p)
        
  

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
