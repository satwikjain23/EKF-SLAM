#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from race.msg import covariance

car_coordinate = [0, 0]
prev = [0, 0]
total = []
pres = []
diff = 0

def call(data):
    global total
    total=data.c



count = 0

def main():
    rospy.init_node("ConeMarker")
    marker_pub = rospy.Publisher("/marker_visualization", MarkerArray, queue_size=1)
    rate = rospy.Rate(1)
    rospy.Subscriber("/covariance_ellipse", covariance, call)
    print(1)
    while not rospy.is_shutdown():
      
        markerArraylala = MarkerArray()

        global count,total
        print(total)
        count=0
        for i in range(0,len(total),4):
            marker = Marker()

            # Set the frame ID and timestamp
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            # Set the namespace and ID for this marker
            marker.ns = "basic_shapes_cones"
            marker.id = count
            count += 1

            # Set the marker type
            marker.type = Marker.CYLINDER  # Use Marker.CYLINDER instead of Marker.Cylinder

            # Set the marker action
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.3)

            # Set the pose of the marker
            # print(total[i+2])
            # print(total[i+3])
            print("--------------------")
            marker.pose.position.x = total[i]  # Set the position from the data
            marker.pose.position.y = total[i+1]  # Set the position from the data
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale of the marker
            marker.scale.x = total[i+2]*1
            marker.scale.y = total[i+3]*1# Set scale for y and z to make it visible
            marker.scale.z = 0.4

            # Set the color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Define the points (note that points is not used for cylindrical markers, so you can remove this section)
            # p = Point()
            # p.x = i[0]
            # p.y = i[1]
            # marker.points.append(p)

            markerArraylala.markers.append(marker)    
        marker_pub.publish(markerArraylala)

        # Cycle between different shapes
        rate.sleep()

if __name__ == "__main__":
    main()
