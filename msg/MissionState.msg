# General mission related information to keep
# track of the state the AA241x handler has on the
# mission.  Additionally contains specific details
# related to the management of the AA241x mission
# (namely offset to Lake Lag coordinate and mission score)

# standard header containing the ROS timestamp information
# for the creation of the data
std_msgs/Header header

# the current mission time in [sec] as computed
# from the start of the mission
int32 mission_time

# some enums for the mission state
uint8 MISSION_NOT_STARTED = 0
uint8 MISSION_RUNNING = 1
uint8 MISSION_ENDED = 2
uint8 MISSION_FAILED_OOB = 3
uint8 MISSION_FAILED_OTHER = 8

# field containing the state information
uint8 mission_state

# the offset data between the lake lag coordinates and the vehicle local
# position information
#
# the lake lake position is the local position coordinate + the offset
float32 e_offset
float32 n_offset
float32 u_offset

# the current score based on the team's position estimates
float32 score