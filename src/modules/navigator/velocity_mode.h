/**
 * @file TargetFollower.hpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once

#include "mission_block.h"

class Navigator;

class VelocityMode : public MissionBlock
{
public:

	enum class Mode {
		TAKEOFF,
		VELOCITY
	};

	VelocityMode(Navigator *navigator, const char *name);

	void on_inactive();
    void on_activation();
    void on_active();

private:

    void set_velocity_command(float vx, float vy, float z, float yaw);


    // uORB subscriptions
    int _velocity_sp_subs[ORB_MULTI_MAX_INSTANCES];

    // onboard parameters
	control::BlockParamFloat _param_min_alt;

    Mode mode;
};


