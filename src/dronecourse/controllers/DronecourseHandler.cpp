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
  _mode(DcMode::IDLE),
  _auto_mode(false),
  _follower(_gimbal)
{
}

DronecourseHandler::~DronecourseHandler()
{
  orb_unadvertise(_local_sp_pub);
}

void DronecourseHandler::update()
{
  _gimbal.update();
  switch (_mode)
  {
    case DcMode::POS_CTRL:
      _pos_ctrl.update();
      break;

    case DcMode::FOLLOW:
      _follower.update();
      break;

    case DcMode::MISSION:
      _trajectory_ctrl.update();
    case DcMode::IDLE:
      break;
  }
}

void DronecourseHandler::set_position_command(float x, float y, float z)
{
  matrix::Vector3f pos(x,y,z);
  _pos_ctrl.set_position_command(pos);
}