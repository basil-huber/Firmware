/**
 * @file PGimbalCtrl.hpp
 * Class to send to control camera gimbal
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>





class GimbalCtrl
{

public:

    enum class MODE {
        MANUAL,
        AUTOMATIC
    };

	GimbalCtrl();

    ~GimbalCtrl(){};

    void setAutomatic(){_mode = MODE::AUTOMATIC;};

    void set_command(float roll, float pitch, float yaw);

    void update();

    float get_yaw_command(){return _yaw;};

private:
    MODE _mode;
    float _yaw;

    // uORB
    int _target_pos_sub;
    orb_advert_t _gimbal_command_pub;
    uORB::Subscription<vehicle_attitude_s> _attitude_sub;
    uORB::Subscription<vehicle_local_position_s> _position_sub;

    
};


