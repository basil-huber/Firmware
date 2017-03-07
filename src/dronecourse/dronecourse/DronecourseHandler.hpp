/**
 * @file DronecourseHandler.hpp
 * Class to handle updates of dronecourse related controllers
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include "PositionCtrl.hpp"
#include "TargetFollower.hpp"

class DronecourseHandler
{

public:

	enum class DcMode {
		IDLE,
		POS_CTRL,
		FOLLOW,
		MISSION
	};

	DronecourseHandler();

    ~DronecourseHandler();

	void update(DcMode mode);

    void set_position_command(float x, float y, float z);

private:

	void send_velocity_command(const matrix::Vector3f& vel_command);

    orb_advert_t _velocity_sp_pub;

    PositionCtrl _pos_ctrl;

    TargetFollower _follower;
};
