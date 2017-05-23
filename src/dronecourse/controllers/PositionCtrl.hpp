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
     * Check if goal is reached
     * i.e. if we are close enough to waypoint
     * (acceptance radius define with parameter POS_ACCEPT_RAD)
     *
     * @return  True if distance to goal smaller than POS_ACCEPT_RAD
     */
    virtual bool is_goal_reached();

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
     * Returns goal position (set by set_position_command) with respect to the drone position
     * (vector from the drone to the goal position in local frame)
     * Updated by update
     *
     * @return  goal position in drone centered local frame
     */
    const matrix::Vector3f& get_target_vector() const {return _target_vector;};


protected:

    /**
     * Updates uORB subscriptions:
     * i.e. updates _current_pos with vehicle_local_position uORB message
     */
	void update_subscriptions();


    /**
     * Updates onboard parameters:
     * i.e. update POS_ACCEPT_RAD
     */
    void update_parameters();

private:

    matrix::Vector3f _goal_pos;         //< Goal position in local frame
    matrix::Vector3f _current_pos;      //< Current position in local frame (updated by _update_subscriptions)
    matrix::Vector3f _target_vector;    //< Vector from drone to goal position
    
    // subscriptions
    int _local_pos_sub;

    // onboard parameter handles
    param_t _p_pos_accept_rad;          // handle for acceptance radius for goal position

    // onboard parameter values
    float   _pos_accept_rad;            //< acceptance radius for goal position
};
