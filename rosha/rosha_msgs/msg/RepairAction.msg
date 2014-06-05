#enum of possible repair actions
uint8 REPAIR_ACTION__RESTART = 0
uint8 REPAIR_ACTION__STOP = 1
uint8 REPAIR_ACTION__START = 2
uint8 REPAIR_ACTION__REPLACE = 3
# ... more to come
# 
# ... special team repairs

# id of the robot
int32 robotId

#type of repair to perform
int32 repairActionToPerform

#component to repair ... how to indentify?
string compName
int32 compId
