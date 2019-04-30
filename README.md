# AA241x Mission #

This package contains the nodes to control the mission for Spring 2019's AA241x.  This README is broken down into the following sections:

 - [Getting Started](#getting-started) - to help with getting you up and running with the mission.

 - [Publishers / Subscribers](#publishers--subscribers) - more detailed explanation of the topic and information the mission publishes and subscribes to.

 - [Rules and Equations](#rules-and-equations) - a summary of the rules and equations that define the Spring 2019 AA241x Mission

 - [General Hints](#general-hints) - some additional hints and useful resources for working with ROS and the catkin environment

## Getting Started ##

The following sections will help you get started with the AA241x Mission and using the `aa241x_mission` node and support launch files:
 - [running the mission](#running-the-mission) - how to run the mission node to run the mission related logic as defined in class and [summarized below](#rules-and-equations)

 - [including the mission](#including-the-mission) - how to include the mission launch handling into your own launch files.

 - [communicating with PX4](#communicating-with-px4) - how to start the MavROS node for communication with PX4 on either the Pixhawk 4 or in simulation with gazebo.

### Running the Mission ###

The AA241x Mission management is contained in a single node (at the moment) and can be started either with the `rosrun` command to start just the node, or it can be started with the `mission.launch` launch file using `roslaunch`.

To start the mission with `roslaunch`:

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mission.launch
```

The mission node has a single parameter that can be adjusted: the `mission_index`, which specifies which of the [mission files](#mission-files) to load for the given flight.  This parameter is also exposed through the launch file as a launch file argument.  For example, the following example specifies the `mission_index` argument to be `1` using `roslaunch`:

```sh
roslaunch aa241x_mission mission.launch mission_index:=1
```

#### Including the Mission ####

You may find that you will want to create your own custom launch files to launch all the nodes you may need for successfully executing all parts of the AA241x Mission.  The `mission.launch` file can be included in your own launch file by using the `<include>` tag:

```xml
<include file="$(find aa241x_mission)/launch/mission.launch" >
    <arg name="mission_index" value="1" />
</include>
```

**Note:** the launch file arguments can also be specified with the `<include>` tag as shown above with the `mission_index` argument being specified to a given value.

### Communicating with PX4 ###

This package contains 2 helper launch files for connection to PX4 with both the Pixhawk 4 hardware and the Gazebo software-in-the-loop simulation.  The details on using and including those launch files are below.

#### MavROS to Pixhawk ####

To launch the MavROS node connected to the physical Pixhawk 4, use the `mavros_pixhawk.launch` file:

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mavros_pixhawk.launch
```

Once again, this launch file can be integrated directly into any custom launch files you create with the following include:

```xml
<include file="$(find aa241x_mission)/launch/mavros_pixhawk.launch" />
```

#### MavROS to Gazebo ####

The connection to the software-in-the-loop simulation of PX4 and the Gazebo environment, use the `mavros_gazebo.launch` file:

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mavros_gazebo.launch
```

Once again, this launch file can be integrated directly into any custom launch files you create with the following include:

```xml
<include file="$(find aa241x_mission)/launch/mavros_gazebo.launch" />
```

To demonstrate the inclusion of a launch file within another launch file, the package contains an example launch file called `mission_sim.launch` that launches both the mission and the connection to the Gazebo simulation.  It can be run like any other launch file:

```sh
roslaunch aa241x_mission mission_sim.launch
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

All the settings and behavior of the mission is outlined in the class presentation and this package merely implements those rules.  Here is a quick recap on some of the more important metric:

### Sensor ###

The "sensor" is simulated by the `aa241x_mission` node.  When the drone is within the given mission area, the node will automatically publish the results of the sensor at the specified rate.  The sensor's performance is defined by the following parameters:

 - **Field of View** - The diameter of the field of view of the sensor is given by the following equation: `diameter = 5/7 * h + 28.57` where `h` is the height above ground as defined by the Lake Lag coordinate frame and is bound by `[30, 100]`.

 - **Rate** - The sensor data is published automatically every 3 seconds (1/3 Hz) to the `measurement` topic and contains the information for all the people in view at the time of the measurement.

 - **Noise** - The sensor will return the position of all the people in view with Gaussian noise with a standard deviation of `sigma = 2 + h/50` meters where `h` is the height above ground as defined by the Lake Lag coordinate frame.


### Bounds ###

The bounds to the mission area are given as follows:

 - the diameter of the allowed mission area is `320 m` centered on `(37.4224444, -122.1760917)` (the origin of the Lake Lag coordinate frame)

 - the maximum allowed altitude is `100 m` and the minimum altitude at which the sensor will operate is `30 m`

 - a violation of either the diameter bound (going outside of the Lake Lag area) or the maximum altitude bound will result in a score of `0`

### Objective ###

Your mission objective is to locate as many people as you can within Lake Lag to better than `1 m` accuracy.  To make an estimate of a person's location, you will publish that estimate to the `person_estimate` topic which will allow the mission node to score you live (and allow logging of the data for post processing if needed).  **Note:** only the last estimate will be used, so it is important you timestamp the data or else the first estimate will be the one that is used by the mission score calculator.


## Mission Files ##

Each mission is defined by a `.mission` file in the `missions/` directory.  The `.mission` files contain the (North, East) coordinates of each of the people located throughout Lake Lag for a given mission.  The file structure is as follows:
 - each line contains the `North East` location of a single person (with only a single space between the North component and the East component)
 - there can be as many lines as desired
 - there should be no empty lines in the file or any comments in the file
 - positions can be as integers or as decimal values, positive or negative

Each file must begin with `id` and then be followed by a number (e.g. `id1`) that is treated as the mission index number.  This mission index number can be set as a launch file parameter for the mission launch file or a node level argument for the mission node (depending on how you choose to start the mission).


## General Hints ##

### ROS Hints ###

For a more detailed explanation of ROS, the elements of a package, and some of the code elements, check out the [ROS Tutorials page](http://wiki.ros.org/ROS/Tutorials) that contains a lot of very useful information (tutorials 2-6, 8, 10, and 11 should be read).  In addition to the tutorials, an effort has been made to comment the code to help explain what some of the elements are doing.


### Catkin Make Hints ###

You may find that you will want to have some custom messages of your own.  If that is the case, make sure to read through [this tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) on creating and using custom messages.