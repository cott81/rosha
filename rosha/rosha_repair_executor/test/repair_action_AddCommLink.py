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
    msg.robotId = 12
    #
    # add comm channel
    #
    msg.repairActionToPerform = 4
    msg.compName = "/vrep/MagicCube/localizationInfo_REMOTE"
    msg.compId = -1
    msg.msgType = "vrep_msgs/Pose2D"

    pub.publish(msg)
    sleep(2)
    pub.publish(msg)
    sleep(2)

    msg = RepairAction()
    msg.robotId = 12    
    msg.repairActionToPerform = 4
    msg.compName = "/vrep/MagicCube/localizationInfo_REMOTE2"
    msg.compId = -1
    msg.msgType = "vrep_msgs/Pose2D"
    
    pub.publish(msg)
    sleep(2)
    
    '''
    while not rospy.is_shutdown():
        pub.publish(msg)
        sleep(5)
    '''
