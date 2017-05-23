#include "TrajectoryCtrl.hpp"

TrajectoryCtrl::TrajectoryCtrl(GimbalCtrl& gimbal) :
	PositionCtrl(gimbal),
	_waypoint_index(0)
{
	_waypoints[0] = matrix::Vector3f(-50, 120,-50);
	_waypoints[1] = matrix::Vector3f( 50, 120,-50);
	_waypoints[2] = matrix::Vector3f( 50,-70,-50);
	_waypoints[3] = matrix::Vector3f(-50,-70,-50);

	set_position_command(_waypoints[_waypoint_index]);
}


void TrajectoryCtrl::update()
{
	

	float dist = get_target_vector().norm();

	if(dist < 1.0f)
	{
		_waypoint_index =  (_waypoint_index + 1) % WAYPOINT_COUNT;
		set_position_command(_waypoints[_waypoint_index]);
	}

	PositionCtrl::update();
}