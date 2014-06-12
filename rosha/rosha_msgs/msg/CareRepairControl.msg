# enum of possible repair actions for Care (without bundle actions)
int16 Undefined = -1
int16 StartProcess = 10
int16 StopProcess = 11
int16 KillProcess = 12
int16 ReplaceProcess = 20

# id of the robot
int32 robotId

# type of repair to perform
int16 repairActionToPerform

# component to repair ... how to indentify?
string compName
int32 compId

# replacement infos
RedundanceInfo compToPlace