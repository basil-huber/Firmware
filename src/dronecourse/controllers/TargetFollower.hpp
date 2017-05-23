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
#include "GimbalCtrl.hpp"


class TargetFollower
{


public:
	TargetFollower(GimbalCtrl& gimbal);

    ~TargetFollower(){;};

	void update();

    const matrix::Vector3f& get_velocity_command() const {return _vel_command;};

private:
    void update_subscriptions();
    
    void update_parameters();
    

    PositionCtrl _pos_ctrl;
    GimbalCtrl&  _gimbal;

    // uORB subscriptions and advertisements
    int _target_pos_sub;
    int _local_pos_sub;

    // onboard parameter handles
    param_t _p_pos_gain;
    param_t _p_vel_gain;
    param_t _p_var_thr_x;
    param_t _p_var_thr_y;
    param_t _p_var_thr_z;
    param_t _p_var_thr_vx;
    param_t _p_var_thr_vy;
    param_t _p_var_thr_vz;
    
    // control parameters
    float _pos_gain;
    float _vel_gain;
    float _var_thr_x;
    float _var_thr_y;
    float _var_thr_z;
    float _var_thr_vx;
    float _var_thr_vy;
    float _var_thr_vz;

    // navigation variables
    bool _has_target_pos_lock;
    bool _has_target_vel_lock;
    bool _has_target_pos_lock_old;
    bool _has_target_vel_lock_old;
    uint64_t _last_lock_time;

    matrix::Vector3f _current_pos;
    matrix::Vector3f _current_vel;
    float            _current_yaw;
    matrix::Vector3f _target_pos;
    matrix::Vector3f _target_vel;
    matrix::Vector3f _vel_command;
};


