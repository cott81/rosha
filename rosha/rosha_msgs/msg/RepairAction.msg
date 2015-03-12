#enum of possible repair actions
uint8 REPAIR_ACTION__RESTART = 0
uint8 REPAIR_ACTION__STOP = 1 # and deactivates diagnosis
uint8 REPAIR_ACTION__START = 2
uint8 REPAIR_ACTION__REPLACE = 3
uint8 REPAIR_ACTION__ADD_COMM_LINK = 4
uint8 REPAIR_ACTION__REMOVE_COMM_LINK = 5
#
# ... special component repairs: magic cube vrep simulation
#
uint8 REPAIR_ACTION__VREP_LOC_REPLACE = 32
uint8 REPAIR_ACTION__VREP_LOC_RECOMPOSE = 33
#
uint8 REPAIR_ACTION__VREP_SLAM_ADAPTER_REPLACE = 40
uint8 REPAIR_ACTION__VREP_SLAM_REMOTE_PROCESS = 41
# 
# ... special team repairs
#
# ... special component repairs: for the car vrep simulation
#
uint8 REPAIR_ACTION__CAR_LOC_REPLACE = 132
uint8 REPAIR_ACTION__CAR_LOC_RECOMPOSE = 133
#
uint8 REPAIR_ACTION__CAR_SLAM_ADAPTER_REPLACE = 140
uint8 REPAIR_ACTION__CAR_SLAM_REMOTE_PROCESS = 141

# id of the robot
int32 robotId

#type of repair to perform
int32 repairActionToPerform

#component to repair ... how to indentify?
string compName
int32 compId
string msgType	#for channel component 

# id of the failed robot
int32 failedRobotId
