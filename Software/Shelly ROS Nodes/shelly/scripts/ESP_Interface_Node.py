#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32

pointcloud = PointCloud()
header = Header()
def callback(value):
    print value
    pointcloud.header.seq +=1
    pointcloud.header.stamp = rospy.get_rostime()
    pointcloud.header.frame_id = "/map"

    pointcloud.points = []
    z_values = [float(numeric_string) for numeric_string in str(value.data).split(",")] # Python list comprehension kung fu
    print z_values
    for num, val in enumerate(z_values):
        point = Point32()
        point.x = num
        point.y = num
        point.z = val
        #print point
        pointcloud.points.append(point)
        #print(pointcloud.points)
    #print "----"

def interface():
    pub1 = rospy.Publisher('/Shelly/header_sent', Header, queue_size=1)
    pub2 = rospy.Publisher('/Shelly/TOF_A/points', PointCloud, queue_size=1)
    rospy.init_node('ESP_Interface_Node', anonymous=True)
    rospy.Subscriber("/Shelly/TOF_A", String, callback)
    rate = rospy.Rate(5) # 10hz

    while not rospy.is_shutdown():
        header.seq +=1
        header.stamp = rospy.get_rostime()
        header.frame_id = "Shelly"
        pub1.publish(header)
        pub2.publish(pointcloud)
        rate.sleep()

if __name__ == '__main__':
    try:
        interface()
    except rospy.ROSInterruptException:
        pass
