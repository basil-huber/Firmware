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
  _has_target_pos_lock(false),
  _has_target_vel_lock(false),
  _vel_command(0.0f, 0.0f, 0.0f)
{
}


void TargetFollower::update()
{
  // check if we received a target position
  bool updated;
  orb_check(_target_pos_sub, &updated);

  if(updated)
  {
    target_position_ned_s target_msg;
    orb_copy(ORB_ID(target_position_ned_filtered), _target_pos_sub, &target_msg);

    // TODO: check if target position and velocity are reliable
    _has_target_pos_lock = true;
    _has_target_vel_lock = true;
  }

  if(_has_target_pos_lock)
  {

    if(_has_target_vel_lock)
    {

    }

  } else {
    // for now, we stay with the same setpoint if we do not have a target lock
  }
}