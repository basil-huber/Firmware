/**
 * @file GimbalCtrl.cpp
 * Class to send to control camera gimbal
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "GimbalCtrl.hpp"
#include <uORB/topics/gimbal_command.h>
#include <uORB/topics/target_position_ned.h>

#include <matrix/math.hpp>
#include <matrix/Euler.hpp>


#include <iostream>

GimbalCtrl::GimbalCtrl() :
  _yaw(0.0f),
  _target_pos_sub(orb_subscribe(ORB_ID(target_position_ned_filtered))),
  _gimbal_command_pub(nullptr),
  _attitude_sub(ORB_ID(vehicle_attitude), 10, 0),
  _position_sub(ORB_ID(vehicle_local_position), 10, 0)
{
  set_command(0.0f, -M_PI/2, _yaw);
}

void GimbalCtrl::set_command(float roll, float pitch, float yaw)
{
  _mode = MODE::MANUAL;
  gimbal_command_s gimbal_command_msg;
  gimbal_command_msg.roll = roll;
  gimbal_command_msg.pitch = pitch;
  _yaw = yaw; 
  int instance;
  orb_publish_auto(ORB_ID(gimbal_command), &_gimbal_command_pub, &gimbal_command_msg, &instance, ORB_PRIO_HIGH);
}

void GimbalCtrl::update()
{
  if(_mode == MODE::AUTOMATIC)
  {

    struct target_position_ned_s target_pos;
    orb_copy(ORB_ID(target_position_ned_filtered), _target_pos_sub, &target_pos);
    // matrix::Vector3f target_pos_lf(target_pos.x, target_pos.y, target_pos.z);
    matrix::Vector3f target_pos_lf(0,30,0);

    // update vehicle attitude and position
    _attitude_sub.update();
    _position_sub.update();
    matrix::Vector3f pos_vehicle(_position_sub.get().x, _position_sub.get().y, _position_sub.get().z);
    matrix::Quaternion<float> att_vehicle(_attitude_sub.get().q);

    // convert to body centered local frame to get yaw commmand
    matrix::Vector3f target_pos_blf = target_pos_lf - pos_vehicle;
    _yaw = atan2(target_pos_blf(1),target_pos_blf(0));
    // convert to body frame to find pitch angle
    // We use current rather than further yaw to find pitch
    matrix::Vector3f target_pos_bf = att_vehicle.conjugate(target_pos_blf);
    float pitch = -atan2(target_pos_bf(2), sqrt(target_pos_bf(0)*target_pos_bf(0) + target_pos_bf(1)*target_pos_bf(1)));

    // Send Gimbal command
    gimbal_command_s gimbal_command_msg;
    // gimbal_command_msg.roll = roll;
    gimbal_command_msg.pitch = pitch;
    int instance;
    orb_publish_auto(ORB_ID(gimbal_command), &_gimbal_command_pub, &gimbal_command_msg, &instance, ORB_PRIO_HIGH);
  }
}