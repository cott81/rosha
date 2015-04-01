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
    # add comm channel
    #
    msg.repairActionToPerform = 4
    msg.compName = "/vrep/carSim/localizationInfo_REMOTE:/vrep/carSim/LaserScanData_REMOTE:/vrep/carSim/InertiaData_REMOTE"
    msg.compId = -1
    msg.msgType = "car_msgs/CarPose2D:car_msgs/CarLaserScanData:car_msgs/InertiaSensorData"

    pub.publish(msg)
    sleep(2)
    pub.publish(msg)
    sleep(2)

    '''
    msg = RepairAction()
    msg.robotId = 12    
    msg.repairActionToPerform = 4
    msg.compName = "/vrep/MagicCube/LaserScanData_REMOTE"
    msg.compId = -1
    msg.msgType = "vrep_msgs/LaserScanData"
    
    pub.publish(msg)
    sleep(2)
    '''

    '''
    while not rospy.is_shutdown():
        pub.publish(msg)
        sleep(5)
    '''
