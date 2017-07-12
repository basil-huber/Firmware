/**
 * @file TargetTracker.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetTracker.hpp"
#include <uORB/topics/target_position_ned.h>
#include <float.h>

TargetTracker::TargetTracker(float dt) :
    _target_position_filtered_pub(nullptr),
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
  // --------------------------------------------------------------------------
	// TODO subscribe to uORB topics:
  //  target_position_ned
  // --------------------------------------------------------------------------
  _target_pos_sub = orb_subscribe(ORB_ID(target_position_ned));

  // set up kalman filter
  const uint8_t m = 6;  // size of state vector
  const uint8_t n = 3;  // size of measurement vector
  matrix::SquareMatrix<float, m> f;
  f.setZero();
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

  bool new_measure = false;
  // ------------------------------------------------------------
  // TODO check if we have a new target_position_ned message
  // and set new_measure accordingly
  // ------------------------------------------------------------
  orb_check(_target_pos_sub, &new_measure);
  if(new_measure)
  {
    // --------------------------------------------------------------------
    // TODO copy message content to a local variable
    // --------------------------------------------------------------------
    struct target_position_ned_s target_pos;
    orb_copy(ORB_ID(target_position_ned),_target_pos_sub, &target_pos);

    matrix::Vector3f target_pos_lf(target_pos.x, target_pos.y, target_pos.z);

    _kf.correct(target_pos_lf);
  }

  // -------------------------------------------------------------------------
  // TODO call publish_filtered_target_position(...) to publish
  //      filtered the filtered target position
  // -------------------------------------------------------------------------
  publish_filtered_target_position(_kf.getStateEstimate(),  _kf.getStateVariances());
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


void TargetTracker::publish_filtered_target_position(const matrix::Vector<float,6>& pos_vel, const matrix::Vector<float,6>& variance)
{
  // -------------------------------------------------------------------------------------------
  // TODO create local variable of type struct target_position_ned_filtered_s
  //      set all fields to 0 and then fill fields
  // -------------------------------------------------------------------------------------------
  struct target_position_ned_s target_pos_filtered;
  memset(&target_pos_filtered, 0, sizeof(target_pos_filtered));     // set all fields to 0

  target_pos_filtered.x       = pos_vel(0);
  target_pos_filtered.y       = pos_vel(1);
  target_pos_filtered.z       = pos_vel(2);
  target_pos_filtered.vx      = pos_vel(3);
  target_pos_filtered.vy      = pos_vel(4);
  target_pos_filtered.vz      = pos_vel(5);
  target_pos_filtered.var_x   = variance(0);
  target_pos_filtered.var_y   = variance(1);
  target_pos_filtered.var_z   = variance(2);
  target_pos_filtered.var_vx  = variance(3);
  target_pos_filtered.var_vy  = variance(4);
  target_pos_filtered.var_vz  = variance(5);
  target_pos_filtered.target_id = 0;

  // -------------------------------------------------------------------------------------------
  // TODO publish your target_position_ned_s message
  // -------------------------------------------------------------------------------------------
  int instance;
  orb_publish_auto(ORB_ID(target_position_ned_filtered), &_target_position_filtered_pub, &target_pos_filtered, &instance, ORB_PRIO_HIGH);
}