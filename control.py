#!/usr/bin/env	python

import rospy
from math import sqrt
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

state = 0

twi = Twist()
twi.linear.x = 0
twi.linear.y = 0
twi.linear.z = 0
twi.angular.x = 0
twi.angular.y = 0
twi.angular.z = 0

        #negative X, positive rotation

def callbackPose(msg):
    global twi

    if (state == 3 or state == 4):    #if we have a lock on
        z = msg.pose.position.z
        x = msg.pose.position.x

        if (z > 0.5):     #distance to target in meters
            speed = 0.5*sqrt(z - 0.5)
            twi.linear.x = speed
        elif (z < 0.45):
            speed = -0.5*sqrt(0.45-z)
            twi.linear.x = speed

        else:
            twi.linear.x = 0

        if (x > 0.07):              #angular distance to center line
            rot = -4*(x-0.07)
            twi.angular.z = rot      #turn clockwise
        elif (x < -0.07):
            rot = 4*(-x-0.07)
            twi.angular.z = rot      #turn counter-clockwise
        elif (x > 0.02):
            rot = -2*(x-0.02)
            twi.angular.z = rot
        elif( x < -0.02):
            rot = 2*(x-0.02)
            twi.angular.z = rot
        else:
            twi.angular.z = 0

        pubTwist.publish(twi)

    else:               #stop the machine if no target is acquired
        twi.linear.x = 0
        twi.angular.z = 0
        pubTwist.publish(twi)

def callbackState(msg):
    global state
    state = msg.data    #deposit tracker state into global variable 'state'


rospy.init_node('control')

subPose = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, callbackPose)  #QR code pose subscriber
subState = rospy.Subscriber('visp_auto_tracker/status', Int8, callbackState)    #tracker status subscriber
pubTwist = rospy.Publisher('delta/cmd_vel', Twist)      #pioneer velocity advertiser
rospy.spin()
