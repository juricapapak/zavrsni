#!/usr/bin/env	python

import rospy
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

linearFuzzy = (-0.15, -0.1, 0, 0.1, 0.15) #upper distance = 0.5, lower = 0.45

angularFuzzy = (-0.5, -0.3, -0.1, 0, 0.1, 0.3, 0.5) #upper distance = 0.02, lower = -0.02


def callbackPose(msg):
    global twi, linearFuzzy, angularFuzzy

    if (state == 3 or state == 4):    #if we have a lock on
        z = msg.pose.position.z
        x = msg.pose.position.x

        if (z > 0.5 and z < 1):     #distance to target in meters
            delta = z - 0.5
            delta *= 4
            ind = (2 - delta)
            twi.linear.x = linearFuzzy[int(round(ind))]
        elif (z < 0.45):
            delta = 0.45-z
            delta *= 4.44
            ind = (2 + delta)
            twi.linear.x = linearFuzzy[int(round(ind))]
        elif (z >= 1):
            twi.linear.x = -0.15
        else:
            twi.linear.x = 0

        if (x > 0.02):              #angular distance to center line
            delta = abs(0.24 - x)
            delta *= 13.6364
            twi.angular.z = angularFuzzy[int(round(delta))]      #turn clockwise
        elif (x < -0.02):
            delta = abs(0.02 + x)
            delta *= 13.6364
            delta += 3
            twi.angular.z = angularFuzzy[int(round(delta))]       #turn counter-clockwise
        else:
            twi.angular.z = 0

        pubTwist.publish(twi)
        #print "X: ", msg.pose.position.x
        #print "Y: ", msg.pose.position.y
        #print "Z: ", msg.pose.position.z
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
