#!/usr/bin/python2.7
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time

class Brain:
    def __init__(self):
        self.pubTwist = None

        self.subBallLocation=None
        self.subBallFound = None
        self.subLineFound = None
        self.instractions = [0,0,0]

        self.ballDistance = 0
        self.ballAngle = 0
        self.ballFound = False

    def ballLocationCallback(self, array):
        if(self.ballFound is True):
            self.ballDistance = array.data[0]
            self.ballAngle = np.arctan(array.data[1]/array.data[0])
            rospy.loginfo(self.ballDistance)

    def BallFoundCallback(self, state):
        self.ballFound = state.data

    def LineFoundCallback(self, data):
        self.instractions = data.data

    def run(self):
        rospy.init_node("BrainNode", anonymous=True)
        self.pubTwist = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size=1000)
        self.subBallFound = rospy.Subscriber("Center_Picture_Ball_Found", Bool, self.BallFoundCallback)
        self.subBallLocation = rospy.Subscriber("Center_BallLocation",Float32MultiArray,self.ballLocationCallback)
        self.subLineFound = rospy.Subscriber("Center_Line_Found",Float32MultiArray,self.LineFoundCallback)
        time.sleep(10)
        self.Routine()
        rospy.spin()

    def Routine(self):
        twst = Twist()
        while True: # infinite loop
            if self.ballFound is False:
                rospy.loginfo("Looking For a Ball")
                while self.ballFound is False:
                    self.instractions = [0, 0, 0]
                    rospy.wait_for_message("Center_Line_Found", Float32MultiArray)
                    twst.angular.z = 0
                    twst.linear.x = 0
                    if(self.instractions[1] ==0 and self.instractions[2]==0):
                        #rospy.loginfo("On Track")
                        twst.angular.z = 0
                    elif self.instractions[1] == 1:
                        #rospy.loginfo("Left")
                        twst.angular.z = 1
                    elif self.instractions[2] == 1:
                        #rospy.loginfo("Right")
                        twst.angular.z = -1
                    twst.linear.x = 0.1*self.instractions[0]
                    self.pubTwist.publish(twst)
                rospy.loginfo("Ball Founded")
                time.sleep(1)
            else:
                while(self.ballFound and (abs(self.ballAngle)>0.2 or self.ballDistance >5)):
                    if(abs(self.ballAngle)>0.2):
                        if(self.ballAngle>0):
                            twst.angular.z = -0.3
                        else:
                            twst.angular.z = 0.3
                    else:
                        twst.angular.z=0
                    if(self.ballDistance>5):
                        twst.linear.x=0.1
                    else:
                        twst.linear.x = 0
                    self.pubTwist.publish(twst)





if __name__ == '__main__':
    try:
        rospy.loginfo("Start ROutine")

        brain = Brain()
        brain.run()
    except rospy.ROSInterruptException:
        pass
