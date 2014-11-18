#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
from debian.changelog import keyvalue

##\author Kevin Watts

##\brief Publishes diagnostic messages for diagnostic aggregator unit test

PKG = 'rosha_diagnostic_aggregator'

import roslib; roslib.load_manifest(PKG)

	
import rospy
from time import sleep

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

if __name__ == '__main__':

    nodeName = 'iceland_SLAM_Adapter'
    rospy.init_node('diag_'+nodeName+'_pub')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray)    
     
    
    array = DiagnosticArray()
    array.status = [
        # GenericAnalyzer prefix1 
        #DiagnosticStatus(0, nodeName, 'msg: all OK', '', [KeyValue('internal_state_valTEST', '4')]),
        #DiagnosticStatus(0, nodeName, 'msg: all OK', '', [KeyValue('CompNotifierType', 'CompHeartBeatInterval'), KeyValue('Interval', '700') ]),
        DiagnosticStatus(0, nodeName, 'msg: xxx', '', [KeyValue('CharacType', 'FlowMsgFreq'), KeyValue('Measurement', '0') ]),
        DiagnosticStatus(0, nodeName, 'msg: xxx', '', [KeyValue('CharacType', 'CompCpu'), KeyValue('Measurement', '700') ]),
        #DiagnosticStatus(0, nodeName, 'msg: xxx', '', [KeyValue('CharacType', 'CompStream'), KeyValue('Measurement', '0') ]),
        #DiagnosticStatus(0, nodeName, 'msg: all OK', '', [KeyValue('CharacType', 'CompMem'), KeyValue('Measurement', '3000') ])
        ]
    #array.header.stamp = rospy.get_rostime()
    array.header.stamp = rospy.Time.now()
    #print rospy.Time.now()
    #print rospy.get_rostime()

    count = 0
    while not rospy.is_shutdown():
        array.header.stamp = rospy.get_rostime()
        print rospy.get_rostime()
        pub.publish(array)
        print "FlowMsgFreq: 0"
        print "CompCpu: 700"
        sleep(1)
        count = count + 1
        if count == 20:
			print "exit"
			exit()
