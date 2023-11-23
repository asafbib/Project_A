#!/usr/bin/python2.7
import rospy
import os
import RPi.GPIO as GPIO
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
        self.motor_is_up = True
        self.pick_lock = True

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


    def Down(self):
        if not self.motor_is_up:
            return
        time.sleep(0.2)
        GPIO.output(11, True)
        time.sleep(1)
        GPIO.output(11, False)
        self.motor_is_up = False
    def Up(self):
        if self.motor_is_up:
            return
        time.sleep(0.5)
        GPIO.output(12, True)
        time.sleep(1.2)
        GPIO.output(12, False)
        self.motor_is_up = True

    def MoveAndLift(self, speed, internal_time):
        if self.pick_lock is True:
            return
        self.pick_lock = True
        current_time = time.time()
        stop_time = current_time + internal_time
        twist = Twist()
        iter = 0
        while current_time < stop_time:
            twist.linear.x = speed
            self.Pub.publish(twist)
            current_time = time.time()
            if iter is 10:
                self.Up()
            iter = iter + 1
        twist.linear.x = 0
        self.Pub.publish(twist)


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
                if self.linear_distance > 100:
                    twst.linear.x = 0.3
                    self.pick_lock = False
                elif self.linear_distance > 45:
                    twst.linear.x = 0.1
                    self.pick_lock = False
                    self.Down()
                elif self.linear_distance > 25:
                    twst.linear.x = 0.03
                else:
                    twst.linear.x = 0
                if (self.angle_deg > 7) or( self.angle_deg < -7):
                    while (time.time()-self.timer_angle) < self.seconds_angle:
                        twst.angular.z = np.sign(self.angle_deg)*(-0.3)
                        rospy.loginfo(self.angle_deg)
                        self.Pub.publish(twst)
                        time.sleep(0.6)
                    twst.angular.z = 0
                self.in_angular_movment = False
                if (twst.angular.z != 0) or (twst.linear.x != 0):
                    self.Pub.publish(twst)
                else:
                    rospy.loginfo("Reach")
                    #raw_input("Pick-up ball.Press Enter To down movmentp")
                    self.MoveAndLift(0.7,1)
                    self.Pub_Reached.publish(True)

if __name__ == '__main__':
    try:
        rospy.init_node("Simple_Brain",anonymous=True)
        instance = Simple()
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(11, GPIO.OUT)
        GPIO.output(12, True)
        GPIO.output(12, False)
        GPIO.output(11, True)
        GPIO.output(11, False)
        instance.Run()
    except rospy.ROSInterruptException:
        pass
