#pragma once

#include "math.h"
#include <Eigen/Dense>

namespace geodetic_trans {

// Geodetic system parameters
static double earthR = 6378137;
static double earthFlattening = 1 / 298.257223563;

/**
 * convert from radian to degrees
 * @param  radians angle in radian
 * @return         angle in degrees
 */
inline double rad2deg(const double radians) {
	return (radians / M_PI) * 180.0;
}

/**
 * convert from degree to radian
 * @param  degrees angle in degree
 * @return         angle in radian
 */
inline double deg2rad(const double degrees) {
	return (degrees / 180.0) * M_PI;
}


/**
 * convert from LLA position to ECEF position
 * @param lat latitude in [deg]
 * @param lon longitude in [deg]
 * @param alt altitude in [deg]
 * @param x   ECEF x position in [m]
 * @param y   ECEF y position in [m]
 * @param z   ECEF z position in [m]
 */
void lla2ecef(const double lat, const double lon, const float alt,
			  float* x, float* y, float* z) {

	double lat_rad = deg2rad(lat);
	double lon_rad = deg2rad(lon);

	double e2 = (2 - earthFlattening) * earthFlattening;
	double r_N = earthR / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
	*x = (r_N + alt) * cos(lat_rad) * cos(lon_rad);
	*y = (r_N + alt) * cos(lat_rad) * sin(lon_rad);
	*z = (r_N * (1 - e2) + alt) * sin(lat_rad);
}



void ecef2ned(const double ref_lat, const double ref_lon, const float ref_alt,
			 const float x, const float y, const float z,
			 float* north, float* east, float* down) {

	// get the reference LLA position as an ECEF coordinate
	float ref_x, ref_y, ref_z;
	lla2ecef(ref_lat, ref_lon, ref_alt, &ref_x, &ref_y, &ref_z);

	// build the rotation matrix from ECEF to NED for the given reference
	// location
	const double s_lat = sin(deg2rad(ref_lat));
    const double s_lon = sin(deg2rad(ref_lon));
    const double c_lat = cos(deg2rad(ref_lat));
    const double c_lon = cos(deg2rad(ref_lon));

    Eigen::Matrix3d rot_ecef2ned;
    rot_ecef2ned(0, 0) = -s_lat * c_lon;
    rot_ecef2ned(0, 1) = -s_lat * s_lon;
    rot_ecef2ned(0, 2) = c_lat;
    rot_ecef2ned(1, 0) = -s_lon;
    rot_ecef2ned(1, 1) = c_lon;
    rot_ecef2ned(1, 2) = 0.0;
    rot_ecef2ned(2, 0) = c_lat * c_lon;
    rot_ecef2ned(2, 1) = c_lat * s_lon;
    rot_ecef2ned(2, 2) = s_lat;

    // get the vector for the reference point to the current location and rotate
    // it
	Eigen::Vector3d ecef_vec, ned_vec;
	ecef_vec << (x - ref_x), (y - ref_y), (z - ref_z);
	ned_vec = rot_ecef2ned * ecef_vec;
	*north = ned_vec(0);
	*east = ned_vec(1);
	*down = -ned_vec(2);
}

void lla2enu(const double ref_lat, const double ref_lon, const float ref_alt,
			 const double lat, const double lon, const float alt,
			 float* east, float* north, float* up) {

	// Geodetic position to local ENU frame
	float x, y, z;
	lla2ecef(lat, lon, alt, &x, &y, &z);

	float aux_north, aux_east, aux_down;
	ecef2ned(ref_lat, ref_lon, ref_alt, x, y, z, &aux_north, &aux_east, &aux_down);

	*east = aux_east;
	*north = aux_north;
	*up = -aux_down;
}




}; // namespace geodetic_trans
