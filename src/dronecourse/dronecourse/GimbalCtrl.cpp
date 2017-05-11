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
  _target_pos_sub(orb_subscribe(ORB_ID(target_position_ned_filtered))),
  _gimbal_command_pub(nullptr),
  _attitude_sub(ORB_ID(vehicle_attitude), 10, 0),
  _position_sub(ORB_ID(vehicle_local_position), 10, 0)
{
  set_command(-M_PI/2, 0.0f);
}

void GimbalCtrl::set_command(float pitch, float yaw)
{
  _mode = MODE::MANUAL;
  gimbal_command_s gimbal_command_msg;
  gimbal_command_msg.yaw = yaw;
  gimbal_command_msg.pitch = pitch;
  int instance;
  orb_publish_auto(ORB_ID(gimbal_command), &_gimbal_command_pub, &gimbal_command_msg, &instance, ORB_PRIO_HIGH);
}

void GimbalCtrl::update()
{
  if(_mode == MODE::AUTOMATIC)
  {

    struct target_position_ned_s target_pos;
    orb_copy(ORB_ID(target_position_ned_filtered), _target_pos_sub, &target_pos);
    matrix::Vector3f target_pos_lf(target_pos.x, target_pos.y, target_pos.z);

    // update vehicle attitude and position
    _attitude_sub.update();
    _position_sub.update();
    matrix::Vector3f pos_vehicle(_position_sub.get().x, _position_sub.get().y, _position_sub.get().z);
    matrix::Quaternion<float> att_vehicle(_attitude_sub.get().q);

    // convert to body centered local frame to get yaw commmand
    const matrix::Vector3f target_dir = (target_pos_lf - pos_vehicle).normalized();

    // find down vector in local frame
    const matrix::Vector3f down_lf = att_vehicle.conjugate_inversed(matrix::Vector3f(0.0f, 0.0f, 1.0f));
    const matrix::Vector3f east_lf = att_vehicle.conjugate_inversed(matrix::Vector3f(0.0f, 1.0f, 0.0f));

    // find normal vector of plane through drone's down vector and truck
    const matrix::Vector3f n = target_dir.cross(down_lf).normalized();

    float yaw = acos(n.dot(east_lf));
    matrix::Vector3f c = east_lf.cross(n);
    if(c.dot(down_lf) < 0)
    {
      yaw = -yaw;
    }

    // find pitch command as angle between down_lf and target_dir
    double alpha = acos(down_lf.dot(target_dir));
    c = target_dir.cross(down_lf);
    if(c.dot(n) > 0)
    {
      alpha = - alpha;
    }
    double pitch = -M_PI/2 + alpha;

    // Send Gimbal command
    gimbal_command_s gimbal_command_msg;
    gimbal_command_msg.pitch = pitch;
    gimbal_command_msg.yaw = yaw;
    int instance;
    orb_publish_auto(ORB_ID(gimbal_command), &_gimbal_command_pub, &gimbal_command_msg, &instance, ORB_PRIO_HIGH);
  }
}
