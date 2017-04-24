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
#include "GimbalCtrl.hpp"
#include "TargetFollower.hpp"
#include "TrajectoryCtrl.hpp"

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

    void set_yaw_command(float yaw);

    GimbalCtrl& gimbal(){return _gimbal;};

private:

	void send_velocity_command(const matrix::Vector3f& vel_command, float yaw_command);
	void send_velocity_command(const matrix::Vector3f& vel_command, float yaw_command, const matrix::Vector3f& pos_command);

    orb_advert_t _local_sp_pub;

    GimbalCtrl _gimbal;
    PositionCtrl _pos_ctrl;
    TargetFollower _follower;
    TrajectoryCtrl _trajectory_ctrl;
};
