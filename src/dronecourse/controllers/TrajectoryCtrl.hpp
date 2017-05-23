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

class TrajectoryCtrl : public PositionCtrl{
public:
	TrajectoryCtrl(GimbalCtrl& gimbal);

	void update();

private:
	int _waypoint_index;
	matrix::Vector3f _waypoints[WAYPOINT_COUNT];
};