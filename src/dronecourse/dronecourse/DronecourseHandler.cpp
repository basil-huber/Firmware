/**
 * @file DronecourseHandler.cpp
 * Class to handle updates of dronecourse related controllers
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "DronecourseHandler.hpp"
#include <uORB/topics/velocity_setpoint.h>
#include <drivers/drv_hrt.h>


#include <iostream>

DronecourseHandler::DronecourseHandler()
{
}

DronecourseHandler::~DronecourseHandler()
{
  orb_unadvertise(_velocity_sp_pub);
}

void DronecourseHandler::update(DcMode mode)
{
  switch (mode)
  {
    case DcMode::POS_CTRL:
      _pos_ctrl.update();
      send_velocity_command(_pos_ctrl.get_velocity_command(), _pos_ctrl.get_yaw_command());
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


void DronecourseHandler::send_velocity_command(const matrix::Vector3f& vel_command, float yaw_command)
{
  velocity_setpoint_s vel_msg;
  vel_msg.vx = vel_command(0);
  vel_msg.vy = vel_command(1);
  vel_msg.vz = vel_command(2);
  vel_msg.yaw = yaw_command;
  vel_msg.timestamp = hrt_absolute_time();

  int instance;
  orb_publish_auto(ORB_ID(velocity_setpoint), &_velocity_sp_pub, &vel_msg, &instance, ORB_PRIO_DEFAULT);
}