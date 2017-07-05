#!/usr/bin/env	python

import rospy
from math import sqrt
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from control.msg import Error

X_LIMIT1 = 0.02     # lower limit of x camera coordinate  (side-to-side object distance)
X_LIMIT2 = 0.07     # middle limit of x camera coordinate
Z_LIMIT = 0.475     # center value of z camera coordinate (horizontal object distance)

state = 0           # state of the tracker node, 3 and 4 mean lock-on acquired

Z = 0               # error metric data
X = 0
err = Error()

twi = Twist()       # initalizing linear and angular robot speeds
twi.linear.x = 0
twi.linear.y = 0
twi.linear.z = 0
twi.angular.x = 0
twi.angular.y = 0
twi.angular.z = 0


def callbackPose(msg):
    """Callback function for the object pose message.
        If tracking is acquired, calculates needed robot speeds for compensation,
        otherwise sets speed to zero."""

    global twi, Z, X

    if (state == 3 or state == 4):          # if target is acquired
        z = msg.pose.position.z
        Z = z
        x = msg.pose.position.x
        X = x

        if (z > Z_LIMIT+0.025:                       # horizontal distance to target in meters
            speed = 0.5*sqrt(z - Z_LIMIT+0.025)
            twi.linear.x = speed
        elif (z < Z_LIMIT-0.025):
            speed = -0.5*sqrt(Z_LIMIT-0.025 - z)
            twi.linear.x = speed
        else:
            twi.linear.x = 0

        if (x > X_LIMIT2):                      # horizontal distance from target to camera centerplane
            rot = -4*(x - X_LIMIT2) - 0.15
            twi.angular.z = rot             # positive distance = turn clockwise
        elif (x < -X_LIMIT2):
            rot = 4*(-x - X_LIMIT2) + 0.15
            twi.angular.z = rot             # negative distance = turn counter-clockwise
        elif (x > X_LIMIT1):
            rot = -3*(x - X_LIMIT1)
            twi.angular.z = rot
        elif(x < -X_LIMIT1):
            rot = 3*(-x - X_LIMIT1)
            twi.angular.z = rot
        else:
            twi.angular.z = 0

        pubTwist.publish(twi)

    else:                                   # stop the machine if no target is acquired
        twi.linear.x = 0
        twi.angular.z = 0
        pubTwist.publish(twi)

def callbackState(msg):
    """Callback function for storing the code tracker data into a global variable
        to be used by the callbackPose function and error metrics"""

    global state
    state = msg.data                        # deposit tracker state into global variable 'state'


rospy.init_node('control')                  # node name in ROS workspace

rate = rospy.Rate(25)                       # rate in hertz

subPose = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, callbackPose)  # QR code pose subscriber
subState = rospy.Subscriber('visp_auto_tracker/status', Int8, callbackState)    # tracker status subscriber
pubTwist = rospy.Publisher('delta/cmd_vel', Twist)      # pioneer velocity publisher
pubError = rospy.Publisher('control_error', Error)      # error metrics publisher

while not rospy.is_shutdown():
    if state == 3 or state == 4:
        err.reference = 0                   # reference value for error metrics
        err.value = X                       # true value
        err.error = 0 - X                   # error value
        pubError.publish(err)
    else:
        err.reference = 0                   # if no tracking is active,
        err.value = 0                       #   all values to zero
        err.error = 0
        pubError.publish(err)
    rate.sleep()                            # sleep for 1/rate seconds
