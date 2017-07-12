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
#include <uORB/Subscription.hpp>

# include "Kalman.hpp"

class TargetTracker
{


public:
	TargetTracker(float dt);

	void update();

private:
    void update_parameters();
    void publish_filtered_target_position(const matrix::Vector<float,6>& pos_vel, const matrix::Vector<float,6>& variance);

    // -------------------------------------------------------------------
    // TODO add uORB subscriptions for
    // target_position_ned
    // -------------------------------------------------------------------
    int _target_pos_sub;    
	
    // uORB publications
    orb_advert_t   _target_position_filtered_pub;

    // kalman filter
    KalmanFilter<6,3> _kf;

    // onboard parameter handles
    param_t _p_kal_sys_noise[6];
    param_t _p_kal_meas_noise[6];
    // onboard parameter values
    float _kal_sys_noise[6];
    float _kal_meas_noise[6];
};


