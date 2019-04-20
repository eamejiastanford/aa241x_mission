/**
 * this file will contain the content for the node that will run the AA241x mission.
 *
 * This should handle publishing the simulated sensor information (TODO: figure
 * out what was decided for the sensor type...)
 *
 * This should also handle any book-keeping information that may be needed for
 * scoring or analyzing the mission performance at the end of the flight.
 */

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>  // eigen functions

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>

#include <aa241x_mission/MissionState.h>
#include <aa241x_mission/SensorMeasurement.h>

#include "geodetic_trans.hpp"



class MissionNode {


public:

	// TODO: constructor
	// TODO: decide how the settings will be passed -> I think I want them as
	// inputs to the constructor instead of having the constructor pull the
	// private NH data
	MissionNode(int mission_index, std::string mission_file);


	// TODO: any services to broadcast (NOTE: need to figure out what services might be neded)

	// the main function to run the node
	int run();

private:

	// node handler
	ros::NodeHandle _nh;

	// mission settings
	int _mission_index = 0;
	std::string _mission_file;

	float _max_alt = 120;	// maximum allowed altitude [m]

	// sensor setting
	float _sensor_min_alt = 10.0;  // [m]
	float _sensor_diameter_mult = 0.1;  // TODO: height * _sensor_d_mult = diameter FOV


	// TODO: populate these values
	double _lake_center_lat = 37.4224444;	// [deg]
	double _lake_center_lon = -122.1760917;	// [deg]
	float _lake_center_alt = 40.0;			// [m]
	float _lake_radius = 170.0f;			// in bound radius from the center [m]

	// mission monitoring
	bool _in_mission = false;		// true if mission is running
	double _mission_time = 0.0;	// time since mission started in [sec]

	// mission "people"
	std::vector<Eigen::Vector3f> _people;

	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;
	bool _offset_computed = false;
	geometry_msgs::PoseStamped _current_local_position;
	mavros_msgs::State _current_state;

	// subscribers
	// TODO: figure out the desired subscriptions
	ros::Subscriber _state_sub;
	ros::Subscriber _gps_sub;
	ros::Subscriber _local_pos_sub;
	ros::Subscriber _gps_vel_sub;

	// publishers
	// TODO: determine what data should be published
	ros::Publisher _lake_local_pub;
	ros::Publisher _measurement_pub;
	ros::Publisher _mission_state_pub;


	// callbacks
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);
	void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void rawGPSVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);


	// helpers
	void loadMission();
	void makeMeasurement();
	void publishMissionState();

};



MissionNode::MissionNode(int mission_index, std::string mission_file) :
_mission_index(mission_index),
_mission_file(mission_file)
{
	// load the mission
	loadMission();

	// subscriptions
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionNode::stateCallback, this);
	_gps_sub = _nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, &MissionNode::gpsCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/local", 10, &MissionNode::localPosCallback, this);
	_gps_vel_sub = _nh.subscribe<geometry_msgs::TwistStamped>("/mavros/global_position/raw/gps_vel", 1, &MissionNode::rawGPSVelCallback, this);
	// TODO: may need to subscribe to the IMU data (?)
	// TODO: may need to subscribe to something that gives me the acceleration commands
	// TODO: need to decide what I want to subscribe to

	// publishering
	_lake_local_pub = _nh.advertise<geometry_msgs::PoseStamped>("aa241x/local_position", 10);
	_measurement_pub = _nh.advertise<aa241x_mission::SensorMeasurement>("measurement", 10);
	_mission_state_pub = _nh.advertise<aa241x_mission::MissionState>("mission_state", 10);

	// TODO: should this node publish local position information??? or should we just use the PX4 local position info?
	// If using the PX4 local position info, have a challenge of needing an offset...
	//
	// in the past we've always had a defined NED frame that is the same every time a vehicle is flown in lake lag...
	//
	// TODO: should see if PX4 has a way to define the GPS coordinate for which to compute local position
}

void MissionNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	_current_state = *msg;
}


// need to be listening to the raw GPS data for knowing the reference point
void MissionNode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	// if we've already handled the offset computation, or don't have a fix yet
	// then continue
	if (_offset_computed || msg->status.status < 0) {
		return;
	}

	// compute the offset using the geodetic transformations
	double lat = msg->latitude;
	double lon = msg->longitude;
	float alt = msg->altitude;
	geodetic_trans::lla2enu(_lake_center_lat, _lake_center_lon, _lake_center_alt,
							lat, lon, alt, &_e_offset, &_n_offset, &_u_offset);

	ROS_INFO("offset computed as: (%0.2f, %0.2f, %0.2f)", _e_offset, _n_offset, _u_offset);

	// NOTE: this assumes that we are catching (0,0) of the local coordinate
	// system correctly
	// TODO: properly test this assumption

	// TODO: compute the offset

	// make as computed
	_offset_computed = true;

}

void MissionNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	// if the offset hasn't been computed, don't publish anything yet
	if (!_offset_computed) {
		return;
	}

	// adjust the position with the offset (NOTE: this keeps the time the same)
	geometry_msgs::PoseStamped local_pos = *msg;
	local_pos.pose.position.x += _e_offset;
	local_pos.pose.position.y += _n_offset;
	local_pos.pose.position.z += _u_offset;

	// publish the data
	_lake_local_pub.publish(local_pos);

	// set the current position information to be the lake local position
	_current_local_position = local_pos;

	// TODO: check the mission conditions
	bool new_state = (_current_state.mode != "OFFBOARD");
	if (new_state != _in_mission) {
		publishMissionState();
	}
	_in_mission = new_state;
}

// need to be listening to the raw GPS velocity for the initialization
void MissionNode::rawGPSVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	// TODO: decide if actually even need the velocity information
}


void MissionNode::loadMission() {
	// TODO: this should load the mission data from a file somewhere
	// need to decide on the format of the mission data
	//
	// I'm thinking each mission is its own file (e.g. m1.mission)
	// within the file, there are N lines for N targets
	// for each target, each line contains the GPS coords of target j

	// TODO: need to make sure that the file exists
	// also need to decide on how we are going to publish the mission data
	std::ifstream infile(_mission_file);
	if (!infile.good()) {
		ROS_ERROR("mission file does not exist!");
	}

	float n, e, d;
	while (infile >> n >> e >> d) {
		// process pair (a,b)
		// TODO: add this information to a data structure
		// TODO: decide on the best data structure for this lookup
		Eigen::Vector3f loc;
		loc << n, e, d;
		_people.push_back(loc);

		ROS_INFO("adding person at: (%0.2f %0.2f %0.2f)", n, e, d);
	}
}

void MissionNode::makeMeasurement() {

	// TODO: calculate FOV of the sensor
	// TODO: check if there are any people in view
	// TODO: for each person in view, get a position measurement
	// TODO: publish a list of position measurements or an empty measurement

}


void MissionNode::publishMissionState() {

	// populate the topic data
	aa241x_mission::MissionState mission_state;
	mission_state.mission_time = _mission_time;

	// TODO: add a state machine here to be able to handle the mission changes
	if (!_in_mission) {
		mission_state.mission_state = aa241x_mission::MissionState::MISSION_NOT_STARTED;
	} else {
		mission_state.mission_state = aa241x_mission::MissionState::MISSION_RUNNING;
	}

	// publish the information
	_mission_state_pub.publish(mission_state);
}


int MissionNode::run() {

	uint8_t counter = 0;  // needed to rate limit the mission state info
	ros::Rate rate(5);  // TODO: set this to the desired sensor rate
	while (ros::ok()) {

		// sensor is disabled until we are in the mission
		// also doesn't work below a given altitude
		if (!_in_mission || _current_local_position.pose.position.z < _sensor_min_alt) {
			// run the ros components
			ros::spinOnce();
			rate.sleep();
			continue;
		}


		// TODO: make a measurement
		makeMeasurement();


		// TODO: publish the mission state information
		// TODO: maybe rate limit this information to a lower rate (e.g. 0.5 Hz)
		// NOTE: if reduce rate, will want to publish the state immediately when it changes
		if (counter % 10 == 0) {
			publishMissionState();
		}

		ros::spinOnce();
		rate.sleep();
		counter++;
	}

	return EXIT_SUCCESS;
}




int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "mission_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings
	int mission_index = 0;
	std::string mission_file;

	private_nh.param("mission_index", mission_index, 0);
	if (!private_nh.getParam("mission_file", mission_file)) {
		ROS_ERROR("failed to get mission file");
	}

	// create the node
	MissionNode node(mission_index, mission_file);

	// run the node
	return node.run();
}