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
  dronecourse_velocity_setpoint_s local_msg;
  local_msg.vx = vel_command(0);
  local_msg.vy = vel_command(1);
  local_msg.vz = vel_command(2);
  local_msg.yaw_valid = (yaw_command == NAN);
  local_msg.yaw = yaw_command;
  local_msg.timestamp = hrt_absolute_time();

  int instance;
  orb_publish_auto(ORB_ID(dronecourse_velocity_setpoint), &_local_sp_pub, &local_msg, &instance, ORB_PRIO_DEFAULT);
}