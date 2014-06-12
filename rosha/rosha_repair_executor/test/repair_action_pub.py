#!/usr/bin/env python

##\author Dominik Kirchner

##\brief Publishes diagnostic messages for diagnostic aggregator unit test
from debian.changelog import keyvalue

PKG = 'rosha_repair_executor'

import roslib; roslib.load_manifest(PKG)


import rospy
from time import sleep

#from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rosha_msgs.msg import RepairAction

if __name__ == '__main__':
    rospy.init_node('repair_action_pub')
    pub = rospy.Publisher('/repair_action', RepairAction)
    
    msg = RepairAction()
    msg.robotId = 12
    msg.repairActionToPerform = 32
    msg.compName = "GPS"
    msg.compId = -1

    
    #pub.publish(msg)
    #sleep(2)
    
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        sleep(10)
    