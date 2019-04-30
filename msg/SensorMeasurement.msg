# A set of measurements from the simulated sensor onboard the drone
# for the AA241x Mission.  A set of measurements consists of noisy
# (North, East) coordinates for each person (identified with a given ID)
# in the current field of view of the sensor.
#
# NOTE: some measurements may contain no seen people, so make sure
# to check the num_measurements field before attempting to 
# access the different fields.

# standard header containing the ROS timestamp information
# for when the data was measured
std_msgs/Header header

$ the number of measurements (N) contained in this message
int32 num_measurements

# list of ids of the N people seen at this time
int32[] id

# list of the north coordinate of the N people seen in [m]
float32[] n

# list of the east coordinate of the N people seen in [m]
float32[] e