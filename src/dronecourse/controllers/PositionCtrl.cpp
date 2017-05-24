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
  // parameter handles
  _p_pos_accept_rad(param_find("POS_ACCEPT_RAD")),
  // parameter values
  _pos_accept_rad(0.0f)
{
    // uORB subscriptions TODO fill subscriptions
}

void PositionCtrl::update()
{
  update_subscriptions();
  update_parameters();

  // calculate target vector (vector from drone to goal position)
  // TODO: calculate target vector
  _target_vector = _goal_pos; // change this line


  // calculate velocity command
  matrix::Vector3f vel_command = matrix::Vector3f(0.0f,0.0f,0.0f); // change this line

  // send velocity command
  send_velocity_command(vel_command);
}


bool PositionCtrl::is_goal_reached()
{
  return (_target_vector.norm() < _pos_accept_rad);
}


void PositionCtrl::update_subscriptions()
{
  // TODO: update _current_pos with contents of vehicle_local_position message
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