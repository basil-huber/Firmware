/**
 * @file TargetDetector.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetDetector.hpp"
#include <uORB/topics/target_position_ned.h>
#include <uORB/topics/target_position_image.h>
#include <uORB/topics/vehicle_attitude.h>

TargetDetector::TargetDetector() :
    _target_position_pub(nullptr),
    _focal_length(IMAGE_WIDTH2 / tan(HFOV_DEFAULT_/2.0f))
{
  // --------------------------------------------------------------------------
	// TODO subscribe to uORB topics:
  //  target_position_image messages, vehicle_attitude, vehicle_local_position
  // --------------------------------------------------------------------------
  _attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
  _position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
  _target_position_image_sub = orb_subscribe(ORB_ID(target_position_image));
}

void TargetDetector::update()
{
  update_subscriptions();

  bool new_measure = false;
  // ------------------------------------------------------------
  // TODO check if we have a new _target_position_image message
  // and set new_measure accordingly
  // ------------------------------------------------------------
  orb_check(_target_position_image_sub, &new_measure);
  if(new_measure)
  {
    // --------------------------------------------------------------------
    // TODO copy message content to a local variable
    // --------------------------------------------------------------------
    struct target_position_image_s target_pos;
    orb_copy(ORB_ID(target_position_image),_target_position_image_sub, &target_pos);

    // --------------------------------------------------------------------
    // TODO find the target location in camera coordinates and store it
    //      as local variable of type matrix::Vector3f
    // --------------------------------------------------------------------
    matrix::Vector3f target_pos_if(target_pos.x - IMAGE_WIDTH2, target_pos.y - IMAGE_HEIGHT2, _focal_length); // target position in image frame
    float scale = target_pos.dist / target_pos_if.norm();
    target_pos_if *= scale;


    // --------------------------------------------------------------------
    // TODO convert to NED camera frame using quaternions from euler angles
    // --------------------------------------------------------------------
    matrix::Quaternion<float> image_rot(matrix::Euler<float>(0.0f, -M_PI/2.0, -M_PI/2.0));
    matrix::Vector3f target_pos_cf = image_rot.conjugate(target_pos_if);

    // --------------------------------------------------------------------
    // TODO convert to the drone's body frame using the gimbal angles
    // --------------------------------------------------------------------
    matrix::Quaternion<float> camera_rot(matrix::Euler<float>(0.0f, target_pos.pitch, target_pos.yaw));
    matrix::Vector3f target_pos_bf = camera_rot.conjugate_inversed(target_pos_cf);

    // --------------------------------------------------------------------
    // TODO convert to NED camera frame using quaternions from euler angles
    // --------------------------------------------------------------------
    matrix::Vector3f target_pos_lf = _att_vehicle.conjugate_inversed(target_pos_bf);
    target_pos_lf += _pos_vehicle;


    // -------------------------------------------------------------------------------------------
    // TODO create local variable of type struct target_position_ned_s
    //      set all fields to 0 and then fill fields x,y and z with target position in local frame
    // -------------------------------------------------------------------------------------------
    struct target_position_ned_s target_pos_ned;
    memset(&target_pos_ned, 0, sizeof(target_pos_ned));     // set all fields to 0
    target_pos_ned.x = target_pos_lf(0);
    target_pos_ned.y = target_pos_lf(1);
    target_pos_ned.z = target_pos_lf(2);


    // -------------------------------------------------------------------------------------------
    // TODO publish your target_position_ned_s message
    // -------------------------------------------------------------------------------------------
    int instance;
    orb_publish_auto(ORB_ID(target_position_ned), &_target_position_pub, &target_pos_ned, &instance, ORB_PRIO_HIGH);
  }
}



void TargetDetector::update_subscriptions()
{
  //--------------------------------------------------------------------------------------
  // TODO update subscriptions for vehicle_attitude as well as for vehicle_local_position
  //   and update the member variables _att_vehicle and _pos_vehicle
  // -------------------------------------------------------------------------------------
  bool updated;
  orb_check(_attitude_sub, &updated);
  if(updated)
  {
    vehicle_attitude_s attitude_msg;
    orb_copy(ORB_ID(vehicle_attitude), _attitude_sub, &attitude_msg);
    _att_vehicle = matrix::Quaternion<float>(attitude_msg.q);
  }

  orb_check(_position_sub, &updated);
  if(updated)
  {
    vehicle_local_position_s pos_msg;
    orb_copy(ORB_ID(vehicle_local_position), _position_sub, &pos_msg);
    _pos_vehicle(0) = pos_msg.x;
    _pos_vehicle(1) = pos_msg.y;
    _pos_vehicle(2) = pos_msg.z;
  }
}