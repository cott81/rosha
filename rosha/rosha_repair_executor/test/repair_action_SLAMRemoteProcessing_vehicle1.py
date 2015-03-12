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
    #pub = rospy.Publisher('/testOut4', RepairAction)
    
    msg = RepairAction()
    msg.robotId = 0    
    #
    # slam remote processing
    #
    msg.repairActionToPerform = 141
    msg.compName = "SLAM"
    msg.compId = -1
    msg.msgType = ""
    msg.failedRobotId = 1
    
    #pub.publish(msg)
    #sleep(2)
    
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        sleep(5)
    
