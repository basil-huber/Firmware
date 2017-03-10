/**
 * @file TargetFollower.cpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "velocity_mode.h"
#include <uORB/topics/position_setpoint_triplet.h>
#include "navigator.h"
#include "../../dronecourse/dronecourse_utils.h"

VelocityMode::VelocityMode(Navigator *navigator, const char *name) :
  MissionBlock(navigator, name),
  _param_min_alt(this, "MIS_TAKEOFF_ALT", false),
  mode(Mode::TAKEOFF)
{
	// subscribe to all instances of velocity setpoint messages
  for(int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++){
    _local_sp_subs[i] = orb_subscribe_multi(ORB_ID(dronecourse_local_setpoint),i);
  }

  // load initial params
  updateParams();
}


void VelocityMode::on_activation(){}


void VelocityMode::on_inactive(){}


void VelocityMode::on_active()
{
  // state machine: in TAKEOFF mode until take off is finished, then switch to VELOCITY mode
  switch(mode)
  {
    case Mode::TAKEOFF:
      set_takeoff_item(&_mission_item, _navigator->get_global_position()->alt + _param_min_alt.get(), 0);
      if(is_mission_item_reached())
      {
        mode = Mode::VELOCITY;
        PX4_INFO("Takeoff finished, switching to velocity mode");
      }
      else
      {
        mission_item_to_position_setpoint(&_mission_item, &_navigator->get_position_setpoint_triplet()->current);
        _navigator->set_position_setpoint_triplet_updated();
        break;
      }
      // no break here

    case Mode::VELOCITY:
      // check if we received any velocity_setpoints
      struct dronecourse_local_setpoint_s local_sp = {};
      bool updated = orb_fetch_all(ORB_ID(dronecourse_local_setpoint), _local_sp_subs, &local_sp, ORB_MULTI_MAX_INSTANCES);

      if(updated)
      {
        set_velocity_command(local_sp);
      }
      break;
  }
}


void VelocityMode::set_velocity_command(dronecourse_local_setpoint_s& local_sp)
{
  // set position_setpoint to target position
    struct position_setpoint_triplet_s* setpoint_triplet = _navigator->get_position_setpoint_triplet();

    // deactivate previous and next waypoint to avoid interference
    setpoint_triplet->previous.valid = false;
    setpoint_triplet->next.valid = false; // the next setpoint is never valid

    setpoint_triplet->current.valid = true;    
    setpoint_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
    
    // set velocity and yaw
    setpoint_triplet->current.velocity_valid = true;
    setpoint_triplet->current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
    setpoint_triplet->current.vx = local_sp.vx;
    setpoint_triplet->current.vy = local_sp.vy;
    setpoint_triplet->current.vz = local_sp.vz;
    setpoint_triplet->current.yaw_valid = true;
    setpoint_triplet->current.yaw = local_sp.yaw;

    // set position (even if not used, for mavlink)
    setpoint_triplet->current.position_valid = false;
    setpoint_triplet->current.x = local_sp.x;
    setpoint_triplet->current.y = local_sp.y;
    setpoint_triplet->current.z = local_sp.z;

    // deactivate all other controls
    setpoint_triplet->current.acceleration_valid = false;
    setpoint_triplet->current.yawspeed_valid = false;
    setpoint_triplet->current.alt_valid = false;

    // tell the navigator that we've updated the position_setpoint
    _navigator->set_position_setpoint_triplet_updated();
}