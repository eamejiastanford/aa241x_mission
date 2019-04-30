# AA241x Mission #

This package contains the nodes to control the mission for Spring 2019's AA241x.

## Getting Started ##

The following sections will help you get started with the AA241x Mission and using the `aa241x_mission` node and support launch files:
 - [running the mission](#running-the-mission) - how to run the mission node to run the mission related logic as defined in class and [summarized below](#rules-and-equations)
 - [including the mission](#including-the-mission) - how to include the mission launch handling into your own launch files.
 - [communicating with PX4](#communicating-with-px4) - how to start the MavROS node for communication with PX4 on either the Pixhawk 4 or in simulation with gazebo.

### Running the Mission ###

TODO: describe how to use the launch files within the mission package.

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mission.launch
```
Has a single parameter: the mission index that defines the set of people that are loaded into the world.  To understand the mission file definition, [see the mission file definition section below](#mission-files).

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mission.launch mission_index:=1
```

helper for launching the mission and the connection to the simulation:

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mission_sim.launch
```

### Including the Mission ###

TODO: describe how to include the mission into a launch file.

```xml
<include file="$(find aa241x_mission)/launch/mission.launch" >
    <arg name="mission_index" value="1" />
</include>
```

### Communicating with PX4 ###

This package contains 2 helper launch files for connection to PX4 with both the Pixhawk 4 hardware and the Gazebo software-in-the-loop simulation.  The details on using and including those launch files are below.

#### MavROS to Pixhawk ####

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mavros_pixhawk.launch
```


```xml
<include file="$(find aa241x_mission)/launch/mavros_pixhawk.launch" />
```

#### MavROS to Gazebo ####

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mavros_gazebo.launch
```

```xml
<include file="$(find aa241x_mission)/launch/mavros_gazebo.launch" />
```

## Publishers / Subscribers ##

### Publishes ###

TODO: describe the publishers

 - `measurement` - this topic, of type [`aa241x_mission::SensorMeasurement`](https://github.com/aa241x/aa241x_mission/blob/master/msg/SensorMeasurement.msg), contains the sensor information for the AA241x mission.  The fields are:
     + `header` - standard header containing the ROS timestamp information for when the data was measured
     + `num_measurements` - the number of measurements (N) contained in this message
     + `id` - list of ids of the N people seen at this time
     + `n` - list of the north coordinate of the N people seen in [m]
     + `e` - list of the east coordinate of the N people seen in [m]

 - `mission_state` - this topic, of type [`aa241x_mission::MissionState`](https://github.com/aa241x/aa241x_mission/blob/master/msg/MissionState.msg), contains general state information for the mission, and the offset needed to go from the ENU frame as computed by PX4 (with origin as the take-off position) to the Lake Lag ENU frame.
     + `header` - standard header containing the ROS timestamp information for the creation of the data
     + `mission_time` - the current mission time in [sec] as computed from the start of the mission
     + `mission_state` - field containing the state information
     + `e_offset` - offset in the East direction in [m]
     + `n_offset` - offset in the North direction in [m]
     + `u_offset` - offset in the Up direction in [m]
     + `score` - the current score based on the team's position estimates

### Subscribes to ###

The mission node subscribes to the following AA241x Mission related topics:

 - `person_estimate` - this topic, of type [`aa241x_mission::PersonEstimate`](https://github.com/aa241x/aa241x_mission/blob/master/msg/PersonEstimate.msg) contains the timestamped (North, East) position estimate of a declared found person.  The fields are:
     + `header` - standard header containing the ROS timestamp information for the time of the estimate
     + `id` - the id of the person for this estimate
     + `n` - the North coordinate of the measurement in [m] in the Lake Lag ENU frame
     + `e` - the East coordinate of the measurement in [m] in the Lake Lag ENU frame

## Rules and Equations ##

All the settings and behavior of the mission is outlined in the class presentation and this package merely implements those rules.

TODO: recap the rules / defining equations


## ROS Hints ##

For a more detailed explanation of ROS, the elements of a package, and some of the code elements, check out the [ROS Tutorials page](http://wiki.ros.org/ROS/Tutorials) that contains a lot of very useful information (tutorials 2-6, 8, 10, and 11 should be read).  In addition to the tutorials, an effort has been made to comment the code to help explain what some of the elements are doing.


## Catkin Make Hints ##

TODO: describe how to add a message to a package.


## Mission Files ##

