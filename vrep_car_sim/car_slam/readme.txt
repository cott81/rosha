ros remapping to use a single vrep_motion with one agent

rosrun vrep_slam vrep_slam_node /vrep/MagicCube12/localizationInfo:=/vrep/MagicCube12/localizationInfo_REMOTE  /vrep/MagicCube12/LaserScanData:=/vrep/MagicCube12/LaserScanData_REMOTE  /vrep/MagicCube12/localizationData:=/vrep/MagicCube12/localizationData_REMOTE __name:=vrep_motion_11

implemented that the robotId (specified in etc/Globals.conf) replaces the XY in the topics!

