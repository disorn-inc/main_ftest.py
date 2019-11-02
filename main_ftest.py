#!/usr/bin/env python
import rospy
import operator
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16,Float64
from sensor_msgs.msg import Joy
from PID import *

# Author: punyapat areerob
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands'
x=0
y=2
tickgreen=1
twist = Twist()
twist2 = Twist()
twist3 = Twist()
joycontrol=0
speed=0
feedback = 50.0
outputpid = 0.0
mode = 1
stage = 0
mission = 1
kp =0.001


#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64,UInt8,Int64
from geometry_msgs.msg import Twist
class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/angle', Float64, self.cbFollowLane, queue_size = 10)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel',Twist, queue_size = 1)
        self.pub_feedback_error = rospy.Publisher('pid/error', Float64,queue_size=10)
        self.pub_feedback_cmd = rospy.Publisher('pid/mv',Float64,queue_size=10)
        self.sub_fin_park = rospy.Subscriber('/pak_or_st',UInt8,self.getPark, queue_size = 10)
        self.pub_pak_or = rospy.Publisher('/pak_or',UInt8,queue_size = 1)
        self.sub_obs = rospy.Subscriber('/obs_de',UInt8,self.getObs, queue_size = 10)
        self.sub_light = rospy.Subscriber('/traffic',Int64,self.getlight, queue_size = 10)
        self.sub_sign = rospy.Subscriber('sign_park',Int64,self.getpark,queue_size=10)
        self.sub_line_dot = rospy.Subscriber('line_dot',Int64,self.get_line_dot,queue_size=10)
        rospy.on_shutdown(self.fnShutDown)
    def cbFollowLane(self, angle):
        global x 
        global twist
        global twist2
        global twist3
        global joycontrol
        global y
        global tickgreen
        global pid
        global mission
        global kp
        feedback=angle.data
        if feedback < 0:
            feedback = 0
        if feedback > 100:
            feedback = 100
        pid.SetPoint = 50.0
        error=50-feedback
        pid.update(feedback)
        outputpid = pid.output
        twist = Twist()
        kp_s = 0.001
        kp_c = 0.3
        #twist.angular.z = outputpid
        if mission == 0:
            twist.angular.z = ((outputpid*2*3.14)/360)*0.7
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            if feedback >= 40 and feedback <=60:
                twist.linear.x = 1
                kp = kp_s
            if feedback < 40:
                kp = kp_c
                twist.linear.x = 1
            if feedback >60:
                kp = kp_c
                twist.linear.x = 1
            if self.light_stage ==0 and tickgreen==1 :
                mission = 1
            if self.sign_stage == 1:
                mission = 2
        if mission == 1:#before parking
            if self.light_stage == 0 :
                twist.linear.x = 0
                twist.angular.z = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
            if self.light_stage == 1:
                mission = 0
                tickgreen = 0
        if mission == 2:#
            if self.linedot == 0:
                twist.angular.z = ((outputpid*2*3.14)/360)*0.7
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                if feedback >= 40 and feedback <=60:
                    twist.linear.x = 1
                    kp = kp_s
                if feedback < 40:
                    kp = kp_c
                    twist.linear.x = 1
                if feedback >60:
                    kp = kp_c
                    twist.linear.x = 1
            if self.linedot ==1:
                if self.obs_seach == 1:
                    twist.angular.z = ((outputpid*2*3.14)/360)*0.7
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    if feedback >= 40 and feedback <=60:
                        twist.linear.x = 1
                        kp = kp_s
                    if feedback < 40:
                        kp = kp_c
                        twist.linear.x = 1
                    if feedback >60:
                        kp = kp_c
                        twist.linear.x = 1
                if self.obs_seach == 0:
                    self.pub_pak_or.publish(1)
        self.pub_cmd_vel.publish(twist)
    def getPark(self,pak_or_st):
            self.park_fin = pak_or_st.data
    def getObs(self,obs_de):
            self.obs_seach = obs_de.data
    def getlight(self,traffic):
            self.light_stage = traffic.data
    def getpark(self,sign_park):
            self.sign_stage = sign_park.data
    def get_line_dot(self,line_dot):
            self.linedot = line_dot.data






 


        
        
        
        
        
        
     #self.pub_cmd_vel.publish(twist)
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)


    def main(self):
        rospy.spin()
        #pid = PID(1,0,0)
        #pid.SetPoint=50.0
        #pid.setSampleTime(0.01)
        
        

if __name__ == '__main__':
    rospy.init_node('control_lane')
    pid = PID(kp,0,0)
    node = ControlLane()
    pid.SetPoint=50.0
    pid.setSampleTime(0.01)
    node.main()
    

