#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('/Shelly/header_sent', Header, queue_size=1)
    rospy.init_node('ESP_Interface_Node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    header = Header()

    while not rospy.is_shutdown():
        header.seq +=1
        header.stamp = rospy.get_rostime()
        header.frame_id = "Shelly"
        pub.publish(header)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
