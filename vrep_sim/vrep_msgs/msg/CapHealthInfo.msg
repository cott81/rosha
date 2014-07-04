#
# info on health of a robot capability
#

# name of the capability
string name

# capability has failure
bool hasFailure

# probability of failure
float64 failureProb

# functionalities to realize the capability
FuncHealthInfo[] funcs