#!/usr/bin/python2.7
import rospy
from geometry_msgs.msg import PoseStamped
from time import sleep

LocationsArray = []
index = 0


class LocationsPublisher:
    def __init__(self):
        self.Pub = None

    def Run(self):
        pub_pos = PoseStamped()
        pub_pos.pose.position.x = 0
        pub_pos.pose.position.y = 0
        pub_pos.pose.position.z = 0
        pub_pos.pose.orientation.w = 1
        pub_pos.pose.orientation.z = 0
        pub_pos.header.frame_id = "map"
        self.Pub.publish(pub_pos)


if __name__ == '__main__':
    try:
        rospy.init_node("Save_Locations",anonymous=True)
        instance = LocationsPublisher()
        instance.Pub = rospy.Publisher("/Interupt", PoseStamped, queue_size=1)
        sleep(1)
        instance.Run()
    except rospy.ROSInterruptException:
        pass
