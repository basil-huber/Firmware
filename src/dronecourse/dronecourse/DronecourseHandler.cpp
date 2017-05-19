/**
 * @file DronecourseHandler.cpp
 * Class to handle updates of dronecourse related controllers
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "DronecourseHandler.hpp"
#include <uORB/topics/dronecourse_velocity_setpoint.h>
#include <drivers/drv_hrt.h>


#include <iostream>

DronecourseHandler::DronecourseHandler() :
  _follower(_gimbal)
{
}

DronecourseHandler::~DronecourseHandler()
{
  orb_unadvertise(_local_sp_pub);
}

void DronecourseHandler::update(DcMode mode)
{
  _gimbal.update();
  switch (mode)
  {
    case DcMode::POS_CTRL:
      _pos_ctrl.update();
      send_velocity_command(_pos_ctrl.get_velocity_command(), 0);
      break;

    case DcMode::FOLLOW:
      _follower.update();
      send_velocity_command(_follower.get_velocity_command(), 0);
      break;

    case DcMode::MISSION:
      _trajectory_ctrl.update();
      send_velocity_command(_trajectory_ctrl.get_velocity_command(), 0);
    case DcMode::IDLE:
      break;
  }
}

void DronecourseHandler::set_position_command(float x, float y, float z)
{
  matrix::Vector3f pos(x,y,z);
  _pos_ctrl.set_position_command(pos);
}


void DronecourseHandler::send_velocity_command(const matrix::Vector3f& vel_command,float yaw_command)
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