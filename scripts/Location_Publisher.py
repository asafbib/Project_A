#!/usr/bin/python2.7
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from time import sleep
from std_msgs.msg import Bool


LocationsArray = []
index = 0


class LocationsPublisher:
    def __init__(self):
        self.Sub = None
        self.Pub = None
        self.Sub_reached = None
	self.Pub_pic = None

        self.SubInterupt = None
        self.InsideInterupt = False

    def InteruptCB(self, poseStamp):
        global index
        global LocationsArray
        self.SubInterupt.unregister()
        rospy.loginfo("Interupt!!!!")
        if index == 0:
            index = len(LocationsArray) - 1
        else:
            index -= 1
        pub_pos = PoseStamped()
        pub_pos.pose.position.x = float(poseStamp.pose.position.x)
        pub_pos.pose.position.y = float(poseStamp.pose.position.y)
        pub_pos.pose.position.z = float(poseStamp.pose.position.z)
        pub_pos.pose.orientation.w = float(poseStamp.pose.orientation.w)
        pub_pos.pose.orientation.z = float(poseStamp.pose.orientation.z)
        pub_pos.header.frame_id = "map"
        self.Pub.publish(pub_pos)
        sleep(1)


    def ReachedCB(self, data):
        global index
        pub_pos = PoseStamped()
        pub_pos.pose.position.x = float(LocationsArray[index][0])
        pub_pos.pose.position.y = float(LocationsArray[index][1])
        pub_pos.pose.position.z = float(LocationsArray[index][2])
        pub_pos.pose.orientation.w = float(LocationsArray[index][3])
        pub_pos.pose.orientation.z = float(LocationsArray[index][4])
        pub_pos.header.frame_id = "map"
        self.Pub.publish(pub_pos)
        sleep(1)


    def StatusCB(self, status):
        rospy.loginfo("Recieve Status")
        global index
        global LocationsArray
        if status.status.text == "Goal reached.":
            #raw_input("Press Enter To Continue")
            pub_pos = PoseStamped()
            pub_pos.pose.position.x = float(LocationsArray[index][0])
            pub_pos.pose.position.y = float(LocationsArray[index][1])
            pub_pos.pose.position.z = float(LocationsArray[index][2])
            pub_pos.pose.orientation.w = float(LocationsArray[index][3])
            pub_pos.pose.orientation.z = float(LocationsArray[index][4])
            pub_pos.header.frame_id = "map"
            self.Pub.publish(pub_pos)
            sleep(1)
            print(len(LocationsArray))
            if index == len(LocationsArray) -1:
                index = 0
            else:
                index += 1
            self.SubInterupt = rospy.Subscriber("/Interupt",PoseStamped, self.InteruptCB)

    def Run(self):
        global LocationsArray
        global index
        with open('Locations.txt', 'r') as my_file:
            for line in my_file:
                x, y, z, orientation_w, orientation_z = line.split()
                LocationsArray.append([x, y, z, orientation_w, orientation_z])
        self.Pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.Sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.StatusCB)
        self.SubInterupt = rospy.Subscriber("/Interupt",PoseStamped, self.InteruptCB)
	self.Pub_pic = rospy.Publisher("/Take_Yolo_Picture", Bool, queue_size=1)
        self.Sub_reached = rospy.Subscriber("simple_reached",Bool, self.ReachedCB)
        sleep(2)	
        pub_pos = PoseStamped()
        pub_pos.pose.position.x = float(LocationsArray[index][0])
        pub_pos.pose.position.y = float(LocationsArray[index][1])
        pub_pos.pose.position.z = float(LocationsArray[index][2])
        pub_pos.pose.orientation.z = float(LocationsArray[index][3])
        pub_pos.header.frame_id = "map"
        self.Pub.publish(pub_pos)
        sleep(2)
        index += 1
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node("Save_Locations",anonymous=True)
        instance = LocationsPublisher()
        instance.Run()
    except rospy.ROSInterruptException:
        pass
