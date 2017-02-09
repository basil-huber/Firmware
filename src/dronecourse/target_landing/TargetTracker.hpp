/**
 * @file TargetTracker.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
// #include <poll.h>
#include <px4_posix.h>
#include <uORB/topics/landing_target.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/Subscription.hpp>

# include "Kalman.hpp"


//#include <uORB/uORB.h>

const static float HFOV_DEFAULT_ = 1.0f;       // horizontal field of view [rad]


class TargetTracker
{


public:
	TargetTracker();

	void update();

private:

	void pack_target_position(struct target_position_ned_s& pos_msg, float x, float y, float z, float vx, float vy, float vz);

    px4_pollfd_struct_t _polls[1];      // polling handle to wait for sensor reading
    
    uORB::Subscription<vehicle_attitude_s> _attitude_sub;
    uORB::Subscription<vehicle_local_position_s> _position_sub;
	orb_advert_t 								_target_position_pub;
	orb_advert_t 								_target_position_filtered_pub;
    // camera parameters
    const static uint16_t IMAGE_WIDTH2 = 640/2;	// half image width
    const static uint16_t IMAGE_HEIGHT2 = 480/2;	// half image height
    const float _focal_length;

    // kalman filter
    KalmanFilter<6,3> _kf;

    int _target_num;
};


