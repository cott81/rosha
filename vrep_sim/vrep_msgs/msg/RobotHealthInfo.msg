#
# info on health of this robot
#

#name of the robot
string name 

#robot id
int32 robotId

#robot in failure mode 
bool hasFailure

#probability of the diagnosed failure
float64 failureProb

#robot local failure recovery failed
bool localFailureRecoveryFailed

#robot has unrecoverable failed
bool isUnrecoverable

#capabilities of the robot
CapHealthInfo[] caps 