/**
 * @file TargetTracker.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetTracker.hpp"
#include <matrix/math.hpp>



#include <iostream>

TargetTracker::TargetTracker() :
    _attitude_sub(ORB_ID(vehicle_attitude), 10, 0),
    _position_sub(ORB_ID(vehicle_local_position), 10, 0),
    _focal_length(IMAGE_WIDTH2 / tan(HFOV_DEFAULT_/2.0f))

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

        _kf.correct(target_pos_lf);
  }

	/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */    
 //    px4_poll(_polls, 1, 1000);
	// if (_polls[0].revents & POLLIN) {
        
 //        // update vehicle attitude and position
 //        _attitude_sub.update();
 //        _position_sub.update();

 //        /* copy data to local buffers */
 //        struct landing_target_s landing_target;
 //        orb_copy(ORB_ID(landing_target), _polls[0].fd, &landing_target);

 //        // compute target position in body frame
 //        float x = -(landing_target.angle_y - IMAGE_HEIGHT2);
 //        float y = landing_target.angle_x - IMAGE_WIDTH2;
 //        float dist = landing_target.distance;
 //        float scale = dist / sqrtf(x*x + y*y + _focal_length*_focal_length);
 //        matrix::Vector3f target_pos_bf(x*scale, y*scale, _focal_length*scale);  // target position in bf

 //        // convert to local frame (NED)
 //        matrix::Quaternion<float> att_vehicle(_attitude_sub.get().q);
 //        matrix::Vector3f pos_vehicle(_position_sub.get().x, _position_sub.get().y, _position_sub.get().z);
 //        matrix::Vector3f target_pos_lf = att_vehicle.conjugate_inversed(target_pos_bf);
 //        target_pos_lf += pos_vehicle;

 //        PX4_INFO("---------------------------------------");
 //        PX4_INFO("%f\t%f\t%f",
 //                     (double)pos_vehicle(1),
 //                     (double)pos_vehicle(0),
 //                     (double)pos_vehicle(2));
 //        PX4_INFO("%f\t%f\t%f",
 //                     (double)target_pos_lf(1),
 //                     (double)target_pos_lf(0),
 //                     (double)-target_pos_lf(2));
 //  	}
}
