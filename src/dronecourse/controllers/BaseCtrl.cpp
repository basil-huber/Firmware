/**
 * @file BaseController.cpp
 * Base class for controllers
 * provides send_velocity_command(...) to send commands to mc_pos_control
 * that should be called from update
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "BaseCtrl.hpp"
#include <uORB/topics/dronecourse_velocity_setpoint.h>
#include <drivers/drv_hrt.h>


void BaseCtrl::send_velocity_command(const matrix::Vector3f& vel_command,float yaw_command)
{


  dronecourse_velocity_setpoint_s vel_setpoint;
  vel_setpoint.vx = vel_command(0);
  vel_setpoint.vy = vel_command(1);
  vel_setpoint.vz = vel_command(2);
  vel_setpoint.yaw_valid = (yaw_command == NAN);
  vel_setpoint.yaw = yaw_command;
  vel_setpoint.timestamp = hrt_absolute_time();

  if (_vel_setpoint_pub == nullptr) {
    _vel_setpoint_pub = orb_advertise(ORB_ID(dronecourse_velocity_setpoint), &vel_setpoint);
  } else
  {
	orb_publish(ORB_ID(dronecourse_velocity_setpoint), _vel_setpoint_pub, &vel_setpoint);
  }
}