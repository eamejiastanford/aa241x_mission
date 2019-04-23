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
#include <cmath>
#include <random>
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
	float _sensor_min_alt = 10.0;		// [m]
	float _sensor_diameter_mult = 0.1;	// TODO: get the correct equation
	float _sensor_stddev = 5;			// TODO: populate this number correctly

	// lake specific parameters
	double _lake_center_lat = 37.4224444;	// [deg]
	double _lake_center_lon = -122.1760917;	// [deg]
	float _lake_center_alt = 40.0;			// [m]
	float _lake_radius = 170.0f;			// in bound radius from the center [m]

	// mission monitoring
	bool _in_mission = false;		// true if mission is running
	double _mission_time = 0.0;		// time since mission started in [sec]

	// mission "people"
	std::vector<Eigen::Vector3f> _people;	// the positions of the people in the world

	// random sampling stuff
	std::normal_distribution<float> _pos_distribution;
	std::default_random_engine _generator;

	// offsets to the local NED frame used by PX4
	float _e_offset = NAN;
	float _n_offset = NAN;
	float _u_offset = NAN;
	bool _offset_computed = false;

	// data
	geometry_msgs::PoseStamped _current_local_position;		// most recent local position info
	mavros_msgs::State _current_state;						// most recent state info

	// subscribers
	ros::Subscriber _state_sub;		// pixhawk state
	ros::Subscriber _gps_sub;		// filtered GPS data from the pixhawk
	ros::Subscriber _local_pos_sub;	// pixhawk local position

	// publishers
	ros::Publisher _measurement_pub;	// simulated sensor "measurement"
	ros::Publisher _mission_state_pub;	// the current mission state


	// callbacks
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);
	void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	// helpers

	/**
	 * read in the mission file and load the positions of the people
	 */
	void loadMission();

	/**
	 * virtualization of the sensor
	 * makes the "measurement" to the people and publishes the data of those in
	 * view
	 */
	void makeMeasurement();

	/**
	 * publish the current mission state information
	 * this includes the frame offset from the local ENU frame to the lake lag
	 * ENU frame
	 */
	void publishMissionState();

};



MissionNode::MissionNode(int mission_index, std::string mission_file) :
_mission_index(mission_index),
_mission_file(mission_file),
_pos_distribution(0, _sensor_stddev),
_generator(ros::Time::now().toSec())
{
	// load the mission
	loadMission();

	// subscriptions
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionNode::stateCallback, this);
	_gps_sub = _nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, &MissionNode::gpsCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &MissionNode::localPosCallback, this);

	// advertise publishers
	_measurement_pub = _nh.advertise<aa241x_mission::SensorMeasurement>("measurement", 10);
	_mission_state_pub = _nh.advertise<aa241x_mission::MissionState>("mission_state", 10);
}

void MissionNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	_current_state = *msg;

	// check the mission conditions
	// TODO: determine if there are other conditions
	bool new_state = (_current_state.mode != "OFFBOARD");
	if (new_state != _in_mission) {
		publishMissionState();
	}
	_in_mission = new_state;
}

void MissionNode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	// need to be listening to the raw GPS data for knowing the reference point

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

	// DEBUG
	ROS_INFO("offset computed as: (%0.2f, %0.2f, %0.2f)", _e_offset, _n_offset, _u_offset);

	// NOTE: this assumes that we are catching (0,0) of the local coordinate
	// system correctly
	// TODO: properly test this assumption

	// publish the mission state with this information
	publishMissionState();

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

	// set the current position information to be the lake local position
	_current_local_position = local_pos;
}

void MissionNode::loadMission() {

	// open the mission file (display an error if the file does not exist)
	std::ifstream infile(_mission_file);
	if (!infile.good()) {
		ROS_ERROR("mission file does not exist!");
	}

	// import the data from the mission file into the people vector
	float n, e, d;
	while (infile >> n >> e >> d) {
		// each person is represented by a 3 vector (NED)
		Eigen::Vector3f loc;
		loc << n, e, d;
		_people.push_back(loc);

		// DEBUG
		ROS_INFO("adding person at: (%0.2f %0.2f %0.2f)", n, e, d);
	}
}

void MissionNode::makeMeasurement() {

	// TODO: calculate FOV of the sensor
	float range = 10;  // TODO: actually calculate this value...

	// put the current local position (ENU) into an NED Egien vector
	float n = _current_local_position.pose.position.y;
	float e = _current_local_position.pose.position.x;
	float d = -_current_local_position.pose.position.z;
	Eigen::Vector3f current_pos;
	current_pos << n, e, d;

	// the measurement message
	aa241x_mission::SensorMeasurement meas;
	meas.num_measurements = 0;

	// check if there are any people in view
	for (uint8_t i = 0; i < _people.size(); i++) {
		Eigen::Vector3f pos = _people[i];

		// for each person in view, get a position measurement
		if ((current_pos - pos).norm() <= range) {

			// get the N and E coordinates of the measurement
			n = _pos_distribution(_generator) + pos(0);
			e = _pos_distribution(_generator) + pos(1);

			// add to the message
			meas.num_measurements++;
			meas.id.push_back(i);
			meas.n.push_back(n);
			meas.e.push_back(e);
			meas.u.push_back(0);  // TODO: determine if actually want this
		}
	}

	// publish a list of position measurements or an empty measurement
	_measurement_pub.publish(meas);
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

	// add the offset information
	mission_state.e_offset = _e_offset;
	mission_state.n_offset = _n_offset;
	mission_state.u_offset = _u_offset;

	// publish the information
	_mission_state_pub.publish(mission_state);
}

int MissionNode::run() {

	uint8_t counter = 0;	// needed to rate limit the mission state info
	ros::Rate rate(5);		// TODO: set this to the desired sensor rate
	while (ros::ok()) {

		// sensor is disabled until we are in the mission
		// also doesn't work below a given altitude
		if (!_in_mission || _current_local_position.pose.position.z < _sensor_min_alt) {
			// run the ros components
			ros::spinOnce();
			rate.sleep();
			continue;
		}

		// TODO: check to see if there are any condition for which a measurement
		// would not occur

		// make a measurement
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