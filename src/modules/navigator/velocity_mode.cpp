/**
 * @file TargetFollower.cpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "velocity_mode.h"
#include <uORB/topics/velocity_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include "navigator.h"
#include "../../dronecourse/dronecourse_utils.h"

VelocityMode::VelocityMode(Navigator *navigator, const char *name) :
  NavigatorMode(navigator, name)
{
	// subscribe to all instances of velocity setpoint messages
  for(int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++){
    _velocity_sp_subs[i] = orb_subscribe_multi(ORB_ID(velocity_setpoint),i);    
  }
}


void VelocityMode::on_activation(){}


void VelocityMode::on_inactive(){}


void VelocityMode::on_active()
{
  // check if we received any velocity_setpoints
  struct velocity_setpoint_s velocity_setpoint = {};
  bool updated = orb_fetch_all(ORB_ID(velocity_setpoint), _velocity_sp_subs, &velocity_setpoint, ORB_MULTI_MAX_INSTANCES);

  if(updated)
  {
    set_velocity_command(velocity_setpoint.vx, velocity_setpoint.vy, velocity_setpoint.vz);
  }
}


void VelocityMode::set_velocity_command(float vx, float vy, float vz)
{
  // set position_setpoint to target position
    struct position_setpoint_triplet_s* setpoint_triplet = _navigator->get_position_setpoint_triplet();

    // deactivate previous and next waypoint to avoid interference
    setpoint_triplet->previous.valid = false;
    setpoint_triplet->next.valid = false; // the next setpoint is never valid

    setpoint_triplet->current.valid = true;    
    setpoint_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
    
    // set velocity
    setpoint_triplet->current.velocity_valid = true;
    setpoint_triplet->current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
    setpoint_triplet->current.vx = vx;
    setpoint_triplet->current.vy = vy;
    setpoint_triplet->current.vz = vz;

    // set altitiude
    setpoint_triplet->current.alt_valid = false;

    // deactivate all other controls
    setpoint_triplet->current.position_valid = false;
    setpoint_triplet->current.acceleration_valid = false;
    setpoint_triplet->current.yaw_valid = false;
    setpoint_triplet->current.yawspeed_valid = false;

    // tell the navigator that we've updated the position_setpoint
    _navigator->set_position_setpoint_triplet_updated();
}