/**
 * @file TargetFollower.cpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetFollower.hpp"
#include <drivers/drv_hrt.h>
#include <px4_time.h>
#include <uORB/topics/target_position_ned.h>
#include <uORB/topics/gimbal_command.h>



#include <iostream>

TargetFollower::TargetFollower(GimbalCtrl& gimbal) :
  PositionCtrl(gimbal),
  _target_pos_sub(orb_subscribe(ORB_ID(target_position_ned_filtered))),
  _local_pos_sub(orb_subscribe(ORB_ID(vehicle_local_position))),
  _p_pos_gain(param_find("FOL_POS")),
  _p_var_thr_x(param_find("VAR_THR_X")),
  _p_var_thr_y(param_find("VAR_THR_Y")),
  _p_var_thr_z(param_find("VAR_THR_Z")),
  _p_var_thr_vx(param_find("VAR_THR_VX")),
  _p_var_thr_vy(param_find("VAR_THR_VY")),
  _p_var_thr_vz(param_find("VAR_THR_VZ")),
  _pos_gain(0.0f),
  _vel_gain(0.0f),
  _var_thr_x(0.0f),
  _var_thr_y(0.0f),
  _var_thr_z(0.0f),
  _var_thr_vx(0.0f),
  _var_thr_vy(0.0f),
  _var_thr_vz(0.0f),
  _has_target_pos_lock(false),
  _has_target_vel_lock(false),
  _has_target_pos_lock_old(false),
  _has_target_vel_lock_old(false),
  _current_yaw(0.0f)
{
}


void TargetFollower::update()
{

  update_parameters();
  // _has_target_pos_lock = false;
  // _has_target_vel_lock = false;
  update_subscriptions();

  if(_has_target_pos_lock && _has_target_vel_lock)
  {
    _gimbal.setAutomatic();

    // get position error
    matrix::Vector3f pos_err = _target_pos - _current_pos + matrix::Vector3f(0.0f,0.0f,-2.0f);
    matrix::Vector3f pos_err_vel = _pos_gain * pos_err;

    for(uint8_t i = 0; i < 3; i++)
    {
      if(pos_err_vel(i) > 1){
        pos_err_vel(i) = 1;
      } else if(pos_err_vel(i) < -1){
        pos_err_vel(i) = -1;
      }

    }

    matrix::Vector3f vel_command = _pos_gain * pos_err + _target_vel;
    vel_command(2) *= 0.4f;
    send_velocity_command(vel_command);

  } else {
    _gimbal.set_command((float)(-M_PI/2.0) + 0.6f, _current_yaw + 0.2f);

    if(hrt_absolute_time() - _last_lock_time < 1e6 ){
      // wait where you are, rise slowly
      matrix::Vector3f vel_command(0.0f, 0.0f, 0.1f);
      send_velocity_command(vel_command);
    } else {
      matrix::Vector3f pos_command(0, 50, -40);
      set_position_command(pos_command);
      PositionCtrl::update();
    }
  }
}


bool TargetFollower::is_goal_reached() const
{
  // if we don't have velocity or position lock we didn't reach the goal
  if(!_has_target_pos_lock || !_has_target_vel_lock)
  {
    return false;
  }

  PX4_INFO("vel_error: %.2f   pos_err  %.2f")

  // check if velocity is close enough
  float vel_error = (_target_vel - _current_vel).norm();
  if(vel_error > 0.5f)
  {
    return false;
  }

  // check if velocity is close enough
  float pos_error = (_target_pos - _current_pos).norm();
  if(pos_error > 0.5f)
  {
    return false;
  }

  return true;
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

    _has_target_pos_lock = target_msg.var_x < _var_thr_x && target_msg.var_y < _var_thr_y && target_msg.var_z < _var_thr_z;
    _has_target_vel_lock = target_msg.var_vx < _var_thr_vx && target_msg.var_vy < _var_thr_vy && target_msg.var_vz < _var_thr_vz;
    if(_has_target_vel_lock && _has_target_pos_lock)
    {
      _last_lock_time = hrt_absolute_time();
    }
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
    _current_yaw    = local_pos_msg.yaw;
  }

  if(_has_target_pos_lock && !_has_target_pos_lock_old)
  {
    PX4_WARN("POS_TARGET_LOCK GAINED");
  } else if (!_has_target_pos_lock && _has_target_pos_lock_old)
  {
    PX4_WARN("POS_TARGET_LOCK LOST");
  }

  if(_has_target_vel_lock && !_has_target_vel_lock_old)
  {
    PX4_WARN("VEL_TARGET_LOCK GAINED");
  } else if (!_has_target_vel_lock && _has_target_vel_lock_old)
  {
    PX4_WARN("VEL_TARGET_LOCK LOST");
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

  if(_p_var_thr_x != PARAM_INVALID)
  {
    param_get(_p_var_thr_x, &_var_thr_x);
  } else {
    PX4_WARN("param VAR_THR_X not found");
  }

  if(_p_var_thr_y != PARAM_INVALID)
  {
    param_get(_p_var_thr_y, &_var_thr_y);
  } else {
    PX4_WARN("param VAR_THR_Y not found");
  }

  if(_p_var_thr_z != PARAM_INVALID)
  {
    param_get(_p_var_thr_z, &_var_thr_z);
  } else {
    PX4_WARN("param VAR_THR_Z not found");
  }

  if(_p_var_thr_vx != PARAM_INVALID)
  {
    param_get(_p_var_thr_vx, &_var_thr_vx);
  } else {
    PX4_WARN("param VAR_THR_VX not found");
  }

  if(_p_var_thr_vy != PARAM_INVALID)
  {
    param_get(_p_var_thr_vy, &_var_thr_vy);
  } else {
    PX4_WARN("param VAR_THR_VY not found");
  }

  if(_p_var_thr_vz != PARAM_INVALID)
  {
    param_get(_p_var_thr_vz, &_var_thr_vz);
  } else {
    PX4_WARN("param VAR_THR_VZ not found");
  }
}