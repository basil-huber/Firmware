/**
 * @file TargetFollower.hpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
// #include <poll.h>
#include <px4_posix.h>
//#include <uORB/topics/landing_target.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/Subscription.hpp>

#include "PositionCtrl.hpp"


class TargetFollower
{


public:
	TargetFollower();

    ~TargetFollower(){;};

	void update();

    const matrix::Vector3f& get_velocity_command() const {return _vel_command;};

    float get_yaw_command() const {return _yaw_command;};

private:
    void update_subscriptions();
    
    void update_parameters();
    

    PositionCtrl _pos_ctrl;

    // uORB subscriptions and advertisements
    int _target_pos_sub;
    int _local_pos_sub;

    // onboard parameters
    param_t _p_pos_gain;
    param_t _p_vel_gain;

    // control parameters
    float _pos_gain;
    float _vel_gain;

    // navigation variables
    bool _has_target_pos_lock;
    bool _has_target_vel_lock;
    bool _has_target_pos_lock_old;
    bool _has_target_vel_lock_old;

    matrix::Vector3f _current_pos;
    matrix::Vector3f _current_vel;
    matrix::Vector3f _target_pos;
    matrix::Vector3f _target_vel;
    matrix::Vector3f _vel_command;
    float            _yaw_command;
};


