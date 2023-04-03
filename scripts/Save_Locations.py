#!/usr/bin/python2.7
import rospy
import os
from nav_msgs.msg import Odometry



class SaveLocations:
    def __init__(self):
        self.Sub = None

    def SaveLocationCB(self, odom):
        self.Sub.unregister()
        linear_x = odom.pose.pose.position.x
        linear_y = odom.pose.pose.position.y
        linear_z = odom.pose.pose.position.z
        orientation_w = odom.pose.pose.orientation.w
        orientation_z = odom.pose.pose.orientation.z
        with open('Locations.txt', 'a') as f:
            f.write(str(linear_x) + " " + str(linear_y) + " " + str(linear_z) + " " + str(orientation_w) + " " + str(orientation_z) + "\n")
        rospy.loginfo("Location Saved")



    def Run(self):
        os.system("rm Locations.txt")
        while True:
            raw_input("Press To Save Location")
            self.Sub = rospy.Subscriber("/odom", Odometry, self.SaveLocationCB)

if __name__ == '__main__':
    try:
        rospy.init_node("Save_Locations",anonymous=True)
        instance = SaveLocations()
        instance.Run()
    except rospy.ROSInterruptException:
        pass
