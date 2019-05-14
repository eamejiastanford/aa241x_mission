# Estimate of the location of a person that has been detected as being found
#
# this message type is how estimates will be communicated to the AA241x Mission
# node that handles the computation of the score for a given mission.

# standard header containing the ROS timestamp information for when the data was
# measured
std_msgs/Header header

# the id of the person at this location
int32 id

# the north coordinate of person found in [m]
float32 n

# the east coordinate of person found in [m]
float32 e