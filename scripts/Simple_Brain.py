#!/usr/bin/python2.7
import rospy
import os
import time
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from actionlib_msgs.msg import GoalID
class Simple:
    def __init__(self):
        self.Sub_data = None
        self.Pub_interrupt = None
        self.Pub_Reached = None
	self.Pub = None

        self.ball_ditected = None
        self.seconds_per_degree = 0.5/40

        self.angle_deg = None

        self.seconds_linear = None
        self.seconds_angle = None
        self.timer_angle = None

        self.linear_distance = None

        self.in_angular_movment = False

	

    def CB_data(self, arg):
        if arg.data[2] == 1:
            self.Pub_interrupt.publish(GoalID())
            self.ball_ditected = True
            self.linear_distance = arg.data[0]
            if not self.in_angular_movment:
                self.angle_deg = (arg.data[1]/np.pi)*180
                self.seconds_angle = self.seconds_per_degree
                self.timer_angle = time.time()
                self.in_angular_movment = True
        else:
            self.ball_ditected = False


    def Run(self):
        rospy.loginfo("Start")
        aligned_to_ball = False
	twst = Twist()
	self.stop_flag = False
        self.Pub_Reached = rospy.Publisher("/simple_reached",Bool,queue_size=1000)
        self.Pub_interrupt = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1000)
	self.Pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size=1000)
        self.Sub = rospy.Subscriber("/simple", Float32MultiArray, self.CB_data)
        time.sleep(2)
        while True:
            if self.ball_ditected and self.angle_deg:
                if self.linear_distance > 90:
                    twst.linear.x = 0.2
                elif self.linear_distance > 50:
                    twst.linear.x = 0.1
                elif self.linear_distance > 30:
                    twst.linear.x = 0.07
                else:

                    twst.linear.x = 0
                if (self.angle_deg > 10) or( self.angle_deg < -10):
                    while (time.time()-self.timer_angle) < self.seconds_angle:
                        twst.angular.z = np.sign(self.angle_deg)*(-0.5)
                        rospy.loginfo(self.angle_deg)
                        self.Pub.publish(twst)
                        time.sleep(0.6)
                    twst.angular.z = 0
                self.in_angular_movment = False
                if (twst.angular.z != 0) or (twst.linear.x != 0):
                    self.Pub.publish(twst)
                else:
                    raw_input("Pick-up bakk.Press Enter To Continue")
                    self.Pub_Reached.publish(True)



if __name__ == '__main__':
    try:
        rospy.init_node("Simple_Brain",anonymous=True)
        instance = Simple()
        instance.Run()
    except rospy.ROSInterruptException:
        pass
