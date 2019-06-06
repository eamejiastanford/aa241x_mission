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
#include <aa241x_mission/PersonEstimate.h>
#include <aa241x_mission/CoordinateConversion.h>
#include <aa241x_mission/RequestLandingPosition.h>

#include "geodetic_trans.hpp"



class MissionNode {


public:

	// TODO: constructor
	// TODO: decide how the settings will be passed -> I think I want them as
	// inputs to the constructor instead of having the constructor pull the
	// private NH data
	MissionNode(int mission_index, std::string mission_file);

	// set some optional parameters
	void setLandingGPS(double landing_lat, double landing_lon);

	// TODO: any services to broadcast (NOTE: need to figure out what services might be neded)

	bool serviceGPStoLakeLagENU(aa241x_mission::CoordinateConversion::Request &req,
		aa241x_mission::CoordinateConversion::Response &res);

	/**
	 * service to request the landing position (position of the "truckbed") in
	 * the Lake Lag ENU frame
	 * @param  req service request (empty)
	 * @param  res service response
	 * @return     true if successfully able to provide data
	 */
	bool serviceRequestLandingPosition(aa241x_mission::RequestLandingPosition::Request &req,
		aa241x_mission::RequestLandingPosition::Response &res);

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
	float _sensor_min_h = 30.0;			// min height AGL for the sensor [m]
	float _sensor_max_h = 100.0f;		// max height AGL for the sensor [m]
	float _sensor_d_mult = 5.0f/7.0f;	// multiplier for the equation (*h)
	float _sensor_d_offset = 28.57;		// [m]
	float _sensor_stddev_a = 2;				// min std dev (at height of 50m) [m]
	float _sensor_stddev_b = 1.0f/50.0f;	// scale factor on h [m]

	// lake specific parameters
	double _lake_ctr_lat = 37.4224444;		// [deg]
	double _lake_ctr_lon = -122.1760917;	// [deg]
	float _lake_ctr_alt = 40.0;				// AMSL [m]
	float _lake_ctr_alt_wgs84 = _lake_ctr_alt - 32.060;	// need WGS84 ellipsoid height for GPS data in ROS [m]
	float _lake_radius = 160.0f;			// in bound radius from the center [m]

	// mission monitoring
	bool _in_mission = false;		// true if mission is running
	double _mission_time = 0.0;		// time since mission started in [sec]
	float _mission_score = 0.0;		// the current score

	// mission "people"
	std::vector<Eigen::Vector2f> _people;	// the positions of the people in the world

	// random sampling stuff
	std::default_random_engine _generator;

	// offsets to the local NED frame used by PX4
	float _e_offset = NAN;
	float _n_offset = NAN;
	float _u_offset = NAN;
	bool _lake_offset_computed = false;

	// landing coordinates
	double _landing_lat = 0.0;
	double _landing_lon = 0.0;
	float _landing_e = 0.0f;
	float _landing_n = 0.0f;
	bool _landing_set = false;

	// data
	geometry_msgs::PoseStamped _current_local_position;		// most recent local position info
	mavros_msgs::State _current_state;						// most recent state info

	// subscribers
	ros::Subscriber _state_sub;			// pixhawk state
	ros::Subscriber _gps_sub;			// filtered GPS data from the pixhawk
	ros::Subscriber _local_pos_sub;		// pixhawk local position
	ros::Subscriber _person_found_sub;	// location of the found individual

	// publishers
	ros::Publisher _measurement_pub;	// simulated sensor "measurement"
	ros::Publisher _mission_state_pub;	// the current mission state

	// services
	ros::ServiceServer _coord_conversion_srv;
	ros::ServiceServer _landing_loc_srv;

	// callbacks
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);
	void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void personFoundCallback(const aa241x_mission::PersonEstimate::ConstPtr& msg);

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
_generator(ros::Time::now().toSec())
{
	// load the mission
	loadMission();

	// subscriptions
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionNode::stateCallback, this);
	_gps_sub = _nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, &MissionNode::gpsCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &MissionNode::localPosCallback, this);
	_person_found_sub = _nh.subscribe<aa241x_mission::PersonEstimate>("person_found", 10, &MissionNode::personFoundCallback, this);

	// advertise publishers
	_measurement_pub = _nh.advertise<aa241x_mission::SensorMeasurement>("measurement", 10);
	_mission_state_pub = _nh.advertise<aa241x_mission::MissionState>("mission_state", 10);

	// advertise coordinate conversion and landing location
	_coord_conversion_srv = _nh.advertiseService("gps_to_lake_lag", &MissionNode::serviceGPStoLakeLagENU, this);
	_landing_loc_srv = _nh.advertiseService("lake_lag_landing_loc", &MissionNode::serviceRequestLandingPosition, this);
}

void MissionNode::setLandingGPS(double landing_lat, double landing_lon) {
		_landing_lat = landing_lat;
		_landing_lon = landing_lon;
		_landing_set = true;

		// do the conversion
		float useless;
		geodetic_trans::lla2enu(_lake_ctr_lat, _lake_ctr_lon, _lake_ctr_alt_wgs84,
							landing_lat, landing_lon, 0.0f, &_landing_e, &_landing_n, &useless);
};

void MissionNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	_current_state = *msg;

	// check the mission conditions
	// TODO: determine if there are other conditions
	bool new_state = (_current_state.mode == "OFFBOARD");
	if (new_state != _in_mission) {
		publishMissionState();
	}
	_in_mission = new_state;
}

void MissionNode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	// need to be listening to the raw GPS data for knowing the reference point

	// if we've already handled the offset computation, or don't have a fix yet
	// then continue
	if (_lake_offset_computed || msg->status.status < 0) {
		return;
	}

	// compute the offset using the geodetic transformations
	double lat = msg->latitude;
	double lon = msg->longitude;
	float alt = msg->altitude;
	geodetic_trans::lla2enu(_lake_ctr_lat, _lake_ctr_lon, _lake_ctr_alt_wgs84,
							lat, lon, alt, &_e_offset, &_n_offset, &_u_offset);

	// DEBUG
	ROS_INFO("offset computed as: (%0.2f, %0.2f, %0.2f)", _e_offset, _n_offset, _u_offset);

	// NOTE: this assumes that we are catching (0,0) of the local coordinate
	// system correctly
	// TODO: properly test this assumption

	// publish the mission state with this information
	publishMissionState();

	// make as computed
	_lake_offset_computed = true;

}

void MissionNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	// if the offset hasn't been computed, don't publish anything yet
	if (!_lake_offset_computed) {
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

void MissionNode::personFoundCallback(const aa241x_mission::PersonEstimate::ConstPtr& msg) {
	// TODO: update the score
}

void MissionNode::loadMission() {

	// open the mission file (display an error if the file does not exist)
	std::ifstream infile(_mission_file);
	if (!infile.good()) {
		ROS_ERROR("mission file does not exist!");
	}

	// import the data from the mission file into the people vector
	float n, e;
	while (infile >> n >> e) {
		// each person is represented by a 3 vector (NED)
		Eigen::Vector2f loc;
		loc << n, e;
		_people.push_back(loc);

		// DEBUG
		ROS_INFO("adding person at: (%0.2f %0.2f)", n, e);
	}
}

void MissionNode::makeMeasurement() {

	// put the current local position (ENU) into an NED Egien vector
	float n = _current_local_position.pose.position.y;
	float e = _current_local_position.pose.position.x;
	Eigen::Vector2f current_pos;
	current_pos << n, e;

	// get the height information into a local variable for readibility
	float h = _current_local_position.pose.position.z;

	// calculate FOV of the sensor
	float radius = (_sensor_d_mult * h + _sensor_d_offset) / 2.0f;	// [m]

	// get the sensor distribution based on the equation
	float sensor_std = _sensor_stddev_a + h * _sensor_stddev_b;
	std::normal_distribution<float> pos_distribution(0, sqrt(sensor_std));

	// the measurement message
	aa241x_mission::SensorMeasurement meas;
	meas.header.stamp = ros::Time::now();
	meas.num_measurements = 0;

	// check if there are any people in view
	for (uint8_t i = 0; i < _people.size(); i++) {
		Eigen::Vector2f pos = _people[i];

		// for each person in view, get a position measurement
		if ((current_pos - pos).norm() <= radius) {

			// get the N and E coordinates of the measurement
			n = pos_distribution(_generator) + pos(0);
			e = pos_distribution(_generator) + pos(1);

			// add to the message
			meas.num_measurements++;
			meas.id.push_back(i);
			meas.n.push_back(n);
			meas.e.push_back(e);
		}
	}

	// publish a list of position measurements or an empty measurement
	_measurement_pub.publish(meas);
}

void MissionNode::publishMissionState() {

	// populate the topic data
	aa241x_mission::MissionState mission_state;
	mission_state.header.stamp = ros::Time::now();
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

	// the current score
	mission_state.score = _mission_score;

	// publish the information
	_mission_state_pub.publish(mission_state);
}

int MissionNode::run() {

	uint8_t counter = 0;	// needed to rate limit the mission state info
	ros::Rate rate(1);		// run the loop at 1Hz, which allows mission state at 0.5Hz and measurement at 1/3Hz
	while (ros::ok()) {

		// make a measurement at 1/3 Hz (and if the mission conditions are met)
		float h = _current_local_position.pose.position.z;
		if (_in_mission && h >= _sensor_min_h && h <= _sensor_max_h && counter % 3 == 0) {
			makeMeasurement();
		}

		// publish the mission state information
		// rate limit this information to a lower rate (e.g. 0.5 Hz)
		// NOTE: anything that changes the values in the state will cause the
		// state to be published so that critical data is sent immediately
		if (counter % 2 == 0) {
			publishMissionState();
		}

		// ros handling + increasing the counter to rate limit topics
		ros::spinOnce();
		rate.sleep();
		counter++;
	}

	return EXIT_SUCCESS;
}


bool MissionNode::serviceGPStoLakeLagENU(aa241x_mission::CoordinateConversion::Request &req,
		aa241x_mission::CoordinateConversion::Response &res) {

	// given a gps lat, lon, alt point
	// need to convert it to the Lake Lag frame ENU coordinate

	float lat = req.latitude;
	float lon = req.longitude;
	float alt = req.altitude;
	float pos_e, pos_n, pos_u;

	geodetic_trans::lla2enu(_lake_ctr_lat, _lake_ctr_lon, _lake_ctr_alt_wgs84,
							lat, lon, alt, &pos_e, &pos_n, &pos_u);

	res.east = pos_e;
	res.north = pos_n;
	res.up = pos_u;

	return true;
}

bool MissionNode::serviceRequestLandingPosition(aa241x_mission::RequestLandingPosition::Request &req,
		aa241x_mission::RequestLandingPosition::Response &res) {

	// return the saved information for the landing position of the drone
	// NOTE: if GPS coordinates not set in the launch file -> return false

	if (!_landing_set) {
		return false;
	}

	// the 2D coordinates to the landing location
	res.east = _landing_e;
	res.north = _landing_n;

	return true;
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
	double landing_lat, landing_lon;

	private_nh.param("mission_index", mission_index, 0);
	if (!private_nh.getParam("mission_file", mission_file)) {
		ROS_ERROR("failed to get mission file");
	}

	// create the node
	MissionNode node(mission_index, mission_file);

	// handling the optional parameters
	if (private_nh.getParam("landing_lat", landing_lat) && private_nh.getParam("landing_lon", landing_lon)) {
		node.setLandingGPS(landing_lat, landing_lon);
		ROS_INFO("landing position coordinates: (%0.2f, %0.2f)", landing_lat, landing_lon);
	} else {
		ROS_INFO("[AA241x] no landing position set");
	}

	// run the node
	return node.run();
}