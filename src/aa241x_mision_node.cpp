/**
 * this file will contain the content for the node that will run the AA241x mission.
 *
 * This should handle publishing the simulated sensor information (TODO: figure
 * out what was decided for the sensor type...)
 *
 * This should also handle any book-keeping information that may be needed for
 * scoring or analyzing the mission performance at the end of the flight.
 */



#include <ros/ros.h>




class AA241xMissionNode {


public:

	// TODO: constructor
	// TODO: decide how the settings will be passed -> I think I want them as
	// inputs to the constructor instead of having the constructor pull the
	// private NH data
	AA241xMissionNode(int mission_index);


	// TODO: any services to broadcast (NOTE: need to figure out what services might be neded)

	// the main function to run the node
	int run();

private:

	// node handler
	NodeHandler _nh;

	// mission settings
	int _mission_index;

	// mission monitoring
	bool _in_mission;		// true if mission is running
	double _mission_time;	// time since mission started in [sec]


	// subscribers
	// TODO: figure out the desired subscriptions
	ros::Subscriber _gps_sub;
	ros::Subscriber _gps_vel_sub;

	// publishers
	// TODO: determine what data should be published
	ros::Publisher _measurement_pub;
	ros::Publisher _mission_state_pub;


	// callbacks
	void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void rawGPSVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);


	// helpers
	void loadMission();

};



AA241xMissionNode::AA241xMissionNode(int mission_index) :
_mission_index(mission_index)
{

	// subscriptions
	_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, &AA241xMissionNode::gpsCallback, this);
	_gps_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/global_position/raw/gps_vel", 1, &AA241xMissionNode::rawGPSVelCallback, this);
	// TODO: may need to subscribe to the IMU data (?)
	// TODO: may need to subscribe to something that gives me the acceleration commands
	// TODO: need to decide what I want to subscribe to

	// publishering
	_measurement_pub = nh.advertise<aa241x_mission::Measurement>("measurement", 10);
	_mission_state_pub = nh.advertise<aa241x_mission::MissionState>("mission_state", 10);

	// TODO: should this node publish local position information??? or should we just use the PX4 local position info?
	// If using the PX4 local position info, have a challenge of needing an offset...
	//
	// in the past we've always had a defined NED frame that is the same every time a vehicle is flown in lake lag...
	//
	// TODO: should see if PX4 has a way to define the GPS coordinate for which to compute local position
}


// need to be listening to the raw GPS data for knowing the reference point
void AA241xMissionNode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	// TODO: decide what to do with the GPS information
	//
	// TODO: decide if doing the mission handling in the callback or in the main loop
	// both are valid options, so just need to decide
}

// need to be listening to the raw GPS velocity for the initialization
void AA241xMissionNode::rawGPSVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	// TODO: decide if actually even need the velocity information
}


void AA241xMissionNode::loadMission() {
	// TODO: this should load the mission data from a file somewhere
	// need to decide on the format of the mission data
	//
	// I'm thinking each mission is its own file (e.g. m1.mission)
	// within the file, there are N lines for N targets
	// for each target, each line contains the GPS coords of target j
}


void AA241xMissionNode::run() {

	ros::Rate rate(50);  // TODO: determine the desired rate
	// NOTE: if all work is happening in the callbacks
	// then don't need anything here, just need
	// ros::spin();
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
}




int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "aa241x_mission_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings
	int mission_index = 0;

	private_nh.param("mission_index", mission_index, 0);

	// create the node
	AA241xMissionNode node(mission_index);

	// run the node
	return node.run();
}