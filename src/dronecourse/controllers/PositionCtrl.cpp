/**
 * @file PositionCtrl.cpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "PositionCtrl.hpp"
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/Subscription.hpp>


#include <iostream>

PositionCtrl::PositionCtrl(GimbalCtrl& gimbal) :
  BaseCtrl(gimbal),
  _goal_pos(0.0f,0.0f,0.0f),
  _current_pos(0.0f,0.0f,0.0f),
  // uORB subscriptions
  _local_pos_sub(orb_subscribe(ORB_ID(vehicle_local_position))),
  // parameter handles
  _p_pos_accept_rad(param_find("POS_ACCEPT_RAD")),
  // parameter values
  _pos_accept_rad(0.0f)
{
}

void PositionCtrl::update()
{
  update_subscriptions();
  update_parameters();

  // calculate target vector (vector from drone to goal position)
  _target_vector = _goal_pos - _current_pos;


  // calculate velocity command
  matrix::Vector3f vel_command = 0.3f * _target_vector;

  // send velocity command
  send_velocity_command(vel_command);
}


bool PositionCtrl::is_goal_reached()
{
  return (_target_vector.norm() < _pos_accept_rad);
}


void PositionCtrl::update_subscriptions()
{
  bool updated;
  orb_check(_local_pos_sub, &updated);
  if(updated)
  {
    vehicle_local_position_s local_pos_msg;
    orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos_msg);
    _current_pos(0) = local_pos_msg.x;
    _current_pos(1) = local_pos_msg.y;
    _current_pos(2) = local_pos_msg.z;
  }
}


void PositionCtrl::update_parameters()
{
  if(_p_pos_accept_rad != PARAM_INVALID)
  {
    param_get(_p_pos_accept_rad, &_pos_accept_rad);
  } else {
    PX4_WARN("param POS_ACCEPT_RAD not found");
  }
}