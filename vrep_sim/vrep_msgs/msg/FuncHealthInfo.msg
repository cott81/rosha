#
# info on health of the functionality
#

#enum of failures ... integrate later
#uint8 REPAIR_ACTION__RESTART = 0

# name of the functionality
string name

# functionality has failure
bool hasFailure

# diagnosed type of failure (enum defined above)
int32 failureType

# probability of failure
float64 failureProb

# diagnostic details ... list with failure types needed?
int32[] diagResults_failureType
float64[] diagResults_failureProbs