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



#include <iostream>

TargetTracker::TargetTracker() :
    _attitude_sub(ORB_ID(vehicle_attitude), 10, 0),
    _position_sub(ORB_ID(vehicle_local_position), 10, 0),
    _target_position_pub(nullptr),
    _target_position_filtered_pub(nullptr),
    _focal_length(IMAGE_WIDTH2 / tan(HFOV_DEFAULT_/2.0f)),
    _target_id(0)
{
	// subscribe to landing target messages
	_polls[0].fd = orb_subscribe(ORB_ID(landing_target));
	_polls[0].events = POLLIN;



  // set up kalman filter
  const uint8_t m = 6;  // size of state vector
  const uint8_t n = 3;  // size of measurement vector
  matrix::SquareMatrix<float, m> f;
  f(0,3) = 1.0f;
  f(1,4) = 1.0f;
  f(2,5) = 1.0f;
  float wf[m] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
  matrix::Vector<float,m> w(wf);
  matrix::Matrix<float,n,m> h;
  h(0,0) = 1.0f;
  h(1,1) = 1.0f;
  h(2,2) = 1.0f;
  float vf[n] = {2.0f, 2.0f, 2.0f};
  matrix::Vector<float,n> v(vf);
  float dt = 0.05f;
  _kf.init(f,w,h,v,dt);

}

void TargetTracker::update()
{
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
        struct landing_target_s landing_target;
        orb_copy(ORB_ID(landing_target), _polls[0].fd, &landing_target);

        // compute target position in body frame
        float x = -(landing_target.angle_y - IMAGE_HEIGHT2);
        float y = landing_target.angle_x - IMAGE_WIDTH2;
        float dist = landing_target.distance;
        float scale = dist / sqrtf(x*x + y*y + _focal_length*_focal_length);
        matrix::Vector3f target_pos_bf(x*scale, y*scale, _focal_length*scale);  // target position in bf

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