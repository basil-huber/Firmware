/**
 * @file BaseController.cpp
 * Base class for controllers
 * provides send_velocity_command(...) to send commands to mc_pos_control
 * that should be called from update
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */


#pragma once
#include <px4_posix.h>
#include <matrix/math.hpp>

class BaseController
{

public:
	virtual void update() = 0;

private:
	void send_velocity_command(const matrix::Vector3f& vel_command, float yaw_command = NAN);

    orb_advert_t _local_sp_pub;
};
