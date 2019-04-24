# AA241x Mission #

This package contains the nodes to control the mission for Spring 2019's AA241x.

## Rules and Equations ##

All the settings and behavior of the mission is outlined in the class presentation and this package merely implements those rules.

TODO: recap the rules / defining equations


## ROS Handling ##

TODO: quick summary or something.

### Running the Mission ###

TODO: describe how to use the launch files within the mission package.

### Including the Mission ###

TODO: describe how to include the mission into a launch file.

```xml
<include file="$(find aa241x_mission)/launch/mission.launch" >
    <arg name="mission_index" value="1" />
</include>
```

### MavROS to Pixhawk ###

```xml
<include file="$(find aa241x_mission)/launch/mavros_pixhawk.launch" />
```

### MavROS to Gazebo ###

```xml
<include file="$(find aa241x_mission)/launch/mavros_gazebo.launch" />
```


## Publishers ##

TODO: describe the publishers

 - `measurement` -> `aa241x_mission::SensorMeasurement`
 - `mission_state` -> `aa241x_mission::MissionState`

TODO: maybe describe how to include the message types into other code?
Need to add it to the depends in cmake file and package file.


## Subscribes to ##

TODO: mention the topic(s) and message types that `aa241x_mission` subscribes to.

 - `person_estimate` -> of type `aa241x_mission::PersonEstimate`
 - others?