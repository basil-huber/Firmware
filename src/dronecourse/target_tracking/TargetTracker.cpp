/**
 * @file TargetTracker.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetTracker.hpp"
#include <matrix/math.hpp>
#include <uORB/topics/target_position_ned.h>
#include <float.h>



#include <iostream>

TargetTracker::TargetTracker(float dt) :
    _attitude_sub(ORB_ID(vehicle_attitude), 10, 0),
    _position_sub(ORB_ID(vehicle_local_position), 10, 0),
    _target_position_pub(nullptr),
    _target_position_filtered_pub(nullptr),
    _focal_length(IMAGE_WIDTH2 / tan(HFOV_DEFAULT_/2.0f)),
    _target_id(0),
    _p_kal_sys_noise {param_find("KAL_SYS_NOISE_X"),
                      param_find("KAL_SYS_NOISE_Y"),
                      param_find("KAL_SYS_NOISE_Z"),
                      param_find("KAL_SYS_NOISE_VX"),
                      param_find("KAL_SYS_NOISE_VY"),
                      param_find("KAL_SYS_NOISE_VZ")},
    _p_kal_meas_noise {param_find("KAL_MEAS_NOISE_X"),
                      param_find("KAL_MEAS_NOISE_Y"),
                      param_find("KAL_MEAS_NOISE_Z")},
    _kal_sys_noise {0.0f},
    _kal_meas_noise {0.0f}
{
	// subscribe to target_position_image messages
	_polls[0].fd = orb_subscribe(ORB_ID(target_position_image));
	_polls[0].events = POLLIN;

  // set up kalman filter
  const uint8_t m = 6;  // size of state vector
  const uint8_t n = 3;  // size of measurement vector
  matrix::SquareMatrix<float, m> f;
  f(0,3) = 1.0f;
  f(1,4) = 1.0f;
  f(2,5) = 1.0f;
  matrix::Vector<float,m> w(_kal_sys_noise);
  matrix::Matrix<float,n,m> h;
  h(0,0) = 1.0f;
  h(1,1) = 1.0f;
  h(2,2) = 1.0f;
  matrix::Vector<float,n> v(_kal_meas_noise);
  _kf.init(f,w,h,v,dt);

}

void TargetTracker::update()
{
  update_parameters();
  _kf.predict();

  struct target_position_ned_s pos_msg;
  int instance;
  bool new_measure;
  orb_check(_polls[0].fd, &new_measure);
  if(new_measure)
  {
    // update vehicle attitude and position
    _attitude_sub.update();
    _position_sub.update();

    /* copy data to local buffers */
    struct target_position_image_s target_pos;
    orb_copy(ORB_ID(target_position_image), _polls[0].fd, &target_pos);

    // compute target position in image frame
    matrix::Vector3f target_pos_if(target_pos.x - IMAGE_WIDTH2, target_pos.y - IMAGE_HEIGHT2, _focal_length); // target position in image frame
    float scale = target_pos.dist / target_pos_if.norm();
    target_pos_if *= scale;

    // convert to camera frame (camera's body frame)
    matrix::Quaternion<float> image_rot(matrix::Euler<float>(0.0f, -M_PI/2.0, -M_PI/2.0));
    matrix::Vector3f target_pos_cf = image_rot.conjugate(target_pos_if);

    // convert to body frame
    matrix::Quaternion<float> camera_rot(matrix::Euler<float>(0.0f, target_pos.pitch, target_pos.yaw));
    matrix::Vector3f target_pos_bf = camera_rot.conjugate_inversed(target_pos_cf);

    // convert to local frame (NED)
    matrix::Quaternion<float> att_vehicle(_attitude_sub.get().q);
    matrix::Vector3f pos_vehicle(_position_sub.get().x, _position_sub.get().y, _position_sub.get().z);

    matrix::Vector3f target_pos_lf = att_vehicle.conjugate_inversed(target_pos_bf);
    target_pos_lf += pos_vehicle;


    pack_target_position(pos_msg, target_pos_lf);
    orb_publish_auto(ORB_ID(target_position_ned), &_target_position_pub, &pos_msg, &instance, ORB_PRIO_HIGH);

    _kf.correct(target_pos_lf);
  }

  matrix::Vector<float,6> x_est = _kf.getStateEstimate();
  matrix::Vector<float,6> x_var = _kf.getStateVariances();
  pack_target_position(pos_msg, x_est, x_var);
  orb_publish_auto(ORB_ID(target_position_ned_filtered), &_target_position_filtered_pub, &pos_msg, &instance, ORB_PRIO_HIGH);
}


void TargetTracker::update_parameters()
{
  bool new_sys_noise = false;
  float temp;

  for(uint8_t i = 0; i < 6; i++)
  {
    param_get(_p_kal_sys_noise[i], &temp);
    if(fabsf(temp - _kal_sys_noise[i]) > FLT_EPSILON)
    {
      new_sys_noise = true;
      _kal_sys_noise[i] = temp;
    }
  }

  if(new_sys_noise)
  {
    PX4_INFO("Updating Kalman system noise");
    matrix::Vector<float, 6> w(_kal_sys_noise);
    _kf.setSystemNoise(w);
  }

  bool new_meas_noise = false;
  for(uint8_t i = 0; i < 6; i++)
  {
    param_get(_p_kal_meas_noise[i], &temp);
    if(fabsf(temp - _kal_meas_noise[i]) > FLT_EPSILON)
    {
      new_meas_noise = true;
      _kal_meas_noise[i] = temp;
    }
  }

  if(new_meas_noise)
  {
    PX4_INFO("Updating Kalman measurement noise");
    _kf.setMeasureNoise(matrix::Vector<float,3>(_kal_meas_noise));
  }
}


void TargetTracker::pack_target_position(struct target_position_ned_s& pos_msg, const matrix::Vector3f& pos)
{
  pos_msg.x       = pos(0);
  pos_msg.y       = pos(1);
  pos_msg.z       = pos(2);
  pos_msg.vx      = 0;
  pos_msg.vy      = 0;
  pos_msg.vz      = 0;
  pos_msg.var_x   = 0;
  pos_msg.var_x   = 0;
  pos_msg.var_x   = 0;
  pos_msg.var_vx  = 0;
  pos_msg.var_vy  = 0;
  pos_msg.var_vz  = 0;
  pos_msg.target_id = _target_id;
}

void TargetTracker::pack_target_position(struct target_position_ned_s& pos_msg, const matrix::Vector<float,6>& pos_vel, const matrix::Vector<float,6>& variance)
{
  pos_msg.x       = pos_vel(0);
  pos_msg.y       = pos_vel(1);
  pos_msg.z       = pos_vel(2);
  pos_msg.vx      = pos_vel(3);
  pos_msg.vy      = pos_vel(4);
  pos_msg.vz      = pos_vel(5);
  pos_msg.var_x   = variance(0);
  pos_msg.var_y   = variance(1);
  pos_msg.var_z   = variance(2);
  pos_msg.var_vx  = variance(3);
  pos_msg.var_vy  = variance(4);
  pos_msg.var_vz  = variance(5);

  pos_msg.target_id = _target_id;
}