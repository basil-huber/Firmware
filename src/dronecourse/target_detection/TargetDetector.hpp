/**
 * @file TargetDetector.hpp
 * Class to detect moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include <uORB/Subscription.hpp>
#include <matrix/math.hpp>

const static float HFOV_DEFAULT_ = 1.0f;       // horizontal field of view [rad]


class TargetDetector
{


public:
	TargetDetector();

	void update();

private:
    void update_subscriptions();

    // -------------------------------------------------------------------
    // TODO add uORB subscriptions for
    // vehicle_attitude, vehicle_local_position and target_position_image
    // -------------------------------------------------------------------
    int _attitude_sub;
    int _position_sub;
    int _target_position_image_sub;
    
	
    // uORB publications
    orb_advert_t   _target_position_pub;

    // vehicle attitude and position (from uORB)
    matrix::Quaternion<float> _att_vehicle;
    matrix::Vector3f          _pos_vehicle;

    // camera parameters
    const static uint16_t IMAGE_WIDTH2 = 640/2;	// half image width
    const static uint16_t IMAGE_HEIGHT2 = 480/2;	// half image height
    const float _focal_length;
};


