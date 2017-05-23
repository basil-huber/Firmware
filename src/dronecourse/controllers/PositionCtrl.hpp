/**
 * @file PositionCtrl.hpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include "BaseCtrl.hpp"




class PositionCtrl : public BaseCtrl
{


public:

    /**
     * Constructor
     *
     * @param gimbal     Gimbal controller to control pitch and yaw of gimbal
     */ 
	PositionCtrl(GimbalCtrl& gimbal);

    ~PositionCtrl(){};

    /**
     * Updates the controller
     * i.e. calculates velocity to reach goal position
     * and sends it using BaseCtrl::send_velocity_command(...)
     *
     * @return  drone position in local frame
     */    
	void update();

    /**
     * Sets goal position of this controller
     *
     * @param pos   Goal position in local frame
     */
    void set_position_command(matrix::Vector3f& pos){_goal_pos = pos;};

    /**
     * Returns goal position in local frame
     * (set by set_position_command);
     *
     * @return  drone position in local frame
     */
    const matrix::Vector3f& get_goal_position() const {return _goal_pos;};

    /**
     * Returns drone position in local frame
     * (updated by calling update_subscriptions())
     *
     * @return  drone position in local frame
     */
    const matrix::Vector3f& get_current_position() const {return _current_pos;};

    /**
     * Calculates goal position (set by set_position_command) with respect to the drone position
     * (vector from the drone to the goal position in local frame)
     *
     * @return  goal position in drone centered local frame
     */
    matrix::Vector3f get_target_vector() const;


protected:

    /**
     * Updates uORB subscriptions:
     * i.e. updates _current_pos with vehicle_local_position uORB message
     */
	void update_subscriptions();

private:

    matrix::Vector3f _goal_pos;         //< Goal position in local frame
    matrix::Vector3f _current_pos;      //< current position in local frame (updated by _update_subscriptions)

    // subscriptions
    int _local_pos_sub;
};
