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

PositionCtrl::PositionCtrl() :
  _goal_pos(0.0f,0.0f,0.0f),
  _vel_command(0.0f,0.0f,0.0f),
  _current_pos(0.0f,0.0f,0.0f),
  _local_pos_sub(orb_subscribe(ORB_ID(vehicle_local_position)))
{
}

void PositionCtrl::update()
{
  update_subscriptions();

  // get error
  matrix::Vector3f err = _goal_pos - _current_pos;
  _vel_command = 0.3f * err;
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