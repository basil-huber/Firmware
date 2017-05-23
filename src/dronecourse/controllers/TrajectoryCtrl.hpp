/**
 * @file TrajectoryController.hpp
 * Class to handle predefined flight trajectory
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once

#include <px4_posix.h>
#include "PositionCtrl.hpp"
#include <vector>

#define WAYPOINT_COUNT 4

class TrajectoryCtrl{
public:
	TrajectoryCtrl();

	void update();

	const matrix::Vector3f& get_velocity_command() const {return _pos_ctrl.get_velocity_command();};

private:
	int _waypoint_index;
	PositionCtrl _pos_ctrl;
	matrix::Vector3f _waypoints[WAYPOINT_COUNT];
};