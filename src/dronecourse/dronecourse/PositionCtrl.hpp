/**
 * @file PositionCtrl.hpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include <matrix/math.hpp>




class PositionCtrl
{


public:
	PositionCtrl();

    ~PositionCtrl(){};

	void update();

    void set_yaw_command(float yaw) {_yaw_command = yaw;};

    void set_position_command(matrix::Vector3f pos){_goal_pos = pos;};

    const matrix::Vector3f& get_velocity_command() const {return _vel_command;};

    float get_yaw_command() const {return _yaw_command;};

    const matrix::Vector3f& get_current_position() const {return _current_pos;};

private:

	void update_subscriptions();
	matrix::Vector3f get_distance();

    matrix::Vector3f _goal_pos;
    matrix::Vector3f _vel_command;
    float            _yaw_command;
    matrix::Vector3f _current_pos;

    // subscriptions
    int _local_pos_sub;

    // PID


};


