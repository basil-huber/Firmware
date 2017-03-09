/**
 * @file TargetFollower.cpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetFollower.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/topics/target_position_ned.h>

// #include <matrix/math.hpp>



#include <iostream>

TargetFollower::TargetFollower() :
  _target_pos_sub(orb_subscribe(ORB_ID(target_position_ned_filtered))),
  _local_pos_sub(orb_subscribe(ORB_ID(vehicle_local_position))),
  _p_pos_gain(param_find("FOL_POS")),
  _p_vel_gain(param_find("FOL_VEL")),
  _pos_gain(0.0f),
  _vel_gain(0.0f),
  _has_target_pos_lock(false),
  _has_target_vel_lock(false),
  _has_target_pos_lock_old(false),
  _has_target_vel_lock_old(false),
  _vel_command(0.0f, 0.0f, 0.0f),
  _yaw_command(0.0f)
{
}


void TargetFollower::update()
{

  update_parameters();
  // _has_target_pos_lock = false;
  // _has_target_vel_lock = false;
  update_subscriptions();

  if(_has_target_pos_lock)
  {
      // get error
      matrix::Vector3f pos_err = _target_pos - _current_pos;
      pos_err(2) *= 0.3f;
      matrix::Vector3f vel_err = _target_vel - _current_vel;

      _vel_command = _pos_gain * pos_err + _vel_gain * vel_err;


  } else {
    matrix::Vector3f goal_pos(0, 70, -70);
    _pos_ctrl.set_position_command(goal_pos);
    _pos_ctrl.set_yaw_command(0);
    _pos_ctrl.update();
    _vel_command = _pos_ctrl.get_velocity_command();
  }
}


void TargetFollower::update_subscriptions()
{
  // check if we received a target position
  bool updated;
  orb_check(_target_pos_sub, &updated);

  _has_target_pos_lock_old = _has_target_pos_lock;
  _has_target_vel_lock_old = _has_target_vel_lock;

  if(updated)
  {
    target_position_ned_s target_msg;
    orb_copy(ORB_ID(target_position_ned_filtered), _target_pos_sub, &target_msg);
    _target_pos(0) = target_msg.x;
    _target_pos(1) = target_msg.y;
    _target_pos(2) = target_msg.z;
    _target_vel(0) = target_msg.vx;
    _target_vel(1) = target_msg.vy;
    _target_vel(2) = target_msg.vz;

    _has_target_pos_lock = target_msg.var_x < 70 && target_msg.var_y < 70 && target_msg.var_z < 50;
    _has_target_vel_lock = target_msg.var_vx < 10 && target_msg.var_vy < 10 && target_msg.var_vz < 10;

  }

    orb_check(_local_pos_sub, &updated);
  if(updated)
  {
    vehicle_local_position_s local_pos_msg;
    orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos_msg);
    _current_pos(0) = local_pos_msg.x;
    _current_pos(1) = local_pos_msg.y;
    _current_pos(2) = local_pos_msg.z;
    _current_vel(0) = local_pos_msg.vx;
    _current_vel(1) = local_pos_msg.vy;
    _current_vel(2) = local_pos_msg.vz;
  }

  if(_has_target_pos_lock && !_has_target_pos_lock_old)
  {
    PX4_WARN("POS_TARGET_LOCK GAINED");
  } else if (!_has_target_pos_lock && _has_target_pos_lock_old)
  {
    PX4_WARN("POS_TARGET_LOCK LOST");
  }
}

void TargetFollower::update_parameters()
{
  if(_p_pos_gain != PARAM_INVALID)
  {
    param_get(_p_pos_gain, &_pos_gain);
  } else {
    PX4_WARN("param FOL_POS not found");
  }

  if(_p_vel_gain != PARAM_INVALID)
  {
    param_get(_p_vel_gain, &_vel_gain);
  } else {
    PX4_WARN("param FOL_VEL not found");
  }
}