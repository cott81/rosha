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
    msg.repairActionToPerform = 0
    msg.compName = "test"
    msg.compId = 1
    '''
    array = DiagnosticArray()
    array.status = [

        # OtherAnalyzer for Other
        # "CompFailure_"+iter->GetNodeName()+"_"+iter->GetStateName()
        #DiagnosticStatus(0, 'other1', 'Error', '', [KeyValue('CompFailure_Robot_failure', '0.8')]),
        DiagnosticStatus(2, 'Robot', 'msg: Failure', 'hw_id', [KeyValue('CompFailure_Robot_failure', '0.8')]),
        DiagnosticStatus(0, 'Motion', 'msg: Failure', 'hw_id', [KeyValue('CompFailure_Robot_failure', '0.1')]),
        DiagnosticStatus(2, 'Loc', 'msg: Failure', 'hw_id', [KeyValue('CompFailure_Robot_failure', '0.1')]),
        DiagnosticStatus(2, 'GPS', 'msg: Failure', 'hw_id', [KeyValue('CompFailure_GPS_failure', '0.9'), KeyValue('RootCause_FailureMode_crash', '0.0'), KeyValue('RootCause_FailureMode_deadlock', '0.1'), KeyValue('RootCause_FailureMode_endlessLoop', '0.9'), KeyValue('RootCause_FailureMode_exception', '0.0') ]),
        DiagnosticStatus(0, 'MotionDriver', 'msg: Failure', 'hw_id', [KeyValue('CompFailure_MotionDriver_failure', '0.9'), KeyValue('RootCause_FailureMode_crash', '0.0'), KeyValue('RootCause_FailureMode_deadlock', '0.1'), KeyValue('RootCause_FailureMode_endlessLoop', '0.0'), KeyValue('RootCause_FailureMode_exception', '0.9') ])
        #DiagnosticStatus(0, 'name', 'msg:Error', 'hw_id', [KeyValue('CompFailure_Robot_failure', '0.8'), KeyValue('CompFailure_Motion_failure', '0.7'), KeyValue('CompFailure_Loc_failure', '0.1')])
        ]
    array.header.stamp = rospy.get_rostime()
    '''

    while not rospy.is_shutdown():
        pub.publish(msg)
        sleep(1)
