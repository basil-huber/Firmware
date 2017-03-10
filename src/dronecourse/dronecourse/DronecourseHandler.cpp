/**
 * @file DronecourseHandler.cpp
 * Class to handle updates of dronecourse related controllers
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "DronecourseHandler.hpp"
#include <uORB/topics/dronecourse_local_setpoint.h>
#include <drivers/drv_hrt.h>


#include <iostream>

DronecourseHandler::DronecourseHandler()
{
}

DronecourseHandler::~DronecourseHandler()
{
  orb_unadvertise(_local_sp_pub);
}

void DronecourseHandler::update(DcMode mode)
{
  switch (mode)
  {
    case DcMode::POS_CTRL:
      _pos_ctrl.update();
      send_velocity_command(_pos_ctrl.get_velocity_command(), _pos_ctrl.get_yaw_command(), _pos_ctrl.get_goal_position());
      break;

    case DcMode::FOLLOW:
      _follower.update();
      send_velocity_command(_follower.get_velocity_command(), _follower.get_yaw_command());
      break;

    case DcMode::MISSION:
    case DcMode::IDLE:
      break;
  }
}

void DronecourseHandler::set_position_command(float x, float y, float z)
{
  matrix::Vector3f pos(x,y,z);
  _pos_ctrl.set_position_command(pos);
}


void DronecourseHandler::set_yaw_command(float yaw)
{
  _pos_ctrl.set_yaw_command(yaw);
}


void DronecourseHandler::send_velocity_command(const matrix::Vector3f& vel_command,float yaw_command)
{
  matrix::Vector3f pos_command(NAN, NAN, NAN);
  send_velocity_command(vel_command, yaw_command, pos_command);
}


void DronecourseHandler::send_velocity_command(const matrix::Vector3f& vel_command,float yaw_command, const matrix::Vector3f& pos_command)
{
  dronecourse_local_setpoint_s local_msg;
  local_msg.vx = vel_command(0);
  local_msg.vy = vel_command(1);
  local_msg.vz = vel_command(2);
  local_msg.yaw = yaw_command;
  // position is unused (only for mavlink)
  local_msg.x = pos_command(0);
  local_msg.y = pos_command(1);
  local_msg.z = pos_command(2);
  local_msg.timestamp = hrt_absolute_time();

  int instance;
  orb_publish_auto(ORB_ID(dronecourse_local_setpoint), &_local_sp_pub, &local_msg, &instance, ORB_PRIO_DEFAULT);
}