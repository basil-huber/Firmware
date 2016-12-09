/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file DistanceSensor.hpp
 * DistanceSensor including calibration
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "DistanceSensor.hpp"
#include "conversion/rotation.h"

#include <iostream>

matrix::Dcm<float> sensorOrientationToAttitude(uint8_t orientation);

DistanceSensor::DistanceSensor() :
	_min_distance(0.0f),
	_max_distance(0.0f),
	_current_distance(0.0f),
	_covariance(0.0f),
	_last_update(0),
	_type(0),
	_orientation(0),
	_id(0),
	_isInitialized(false),
	_isActive(false),
	_position_bf()
{
}


void DistanceSensor::initialize(uint8_t id, uint8_t orientation, uint8_t type)
{
	_id = id;
	_orientation = orientation;
	_type = type;
	_isInitialized = true;

	_attitude = sensorOrientationToAttitude(_orientation);
}

void DistanceSensor::setMeasurement(float min_distance,
					float max_distance,
					float current_distance,
					float covariance,
					uint64_t timestamp,
					matrix::Dcm<float> vehicle_attitude,
					Position vehicle_position_lf)
{
	_min_distance = min_distance;
	_max_distance = max_distance;
	_current_distance = current_distance;
	_covariance = covariance;
	_last_update = timestamp;
	_isActive = true;


	// calculate obstacle position in body frame
	_obstacle_position_bf(0) = 0.0f;
	_obstacle_position_bf(1) = 0.0f;
	_obstacle_position_bf(2) = _current_distance;
	_obstacle_position_bf = _attitude * _obstacle_position_bf;
	_obstacle_position_bf += _position_bf;
	
	// calculate obstacle position in local frame (NED)
	_obstacle_position_lf = vehicle_attitude * _obstacle_position_bf;
	_obstacle_position_lf += vehicle_position_lf;
}


typedef matrix::Euler<float> EulerA;

const EulerA rotations[] =
{
	EulerA(0,0,0),	 	 		// MAV_SENSOR_ROTATION_NONE=0, 				Roll: 0, Pitch: 0, Yaw: 0 | */
	EulerA(0,0,0.25*M_PI),		// MAV_SENSOR_ROTATION_YAW_45=1, 			Roll: 0, Pitch: 0, Yaw: 45 | */
   	EulerA(0,0,0.50*M_PI),		// MAV_SENSOR_ROTATION_YAW_90=2, 			Roll: 0, Pitch: 0, Yaw: 90 | */
    EulerA(0,0,0.75*M_PI),		// MAV_SENSOR_ROTATION_YAW_135=3, 			Roll: 0, Pitch: 0, Yaw: 135 | */
   	EulerA(0,0,1.00*M_PI),		// MAV_SENSOR_ROTATION_YAW_180=4, 			Roll: 0, Pitch: 0, Yaw: 180 | */
    EulerA(0,0,1.25*M_PI),		// MAV_SENSOR_ROTATION_YAW_225=5, 			Roll: 0, Pitch: 0, Yaw: 225 | */
    EulerA(0,0,1.50*M_PI),		// MAV_SENSOR_ROTATION_YAW_270=6, 			Roll: 0, Pitch: 0, Yaw: 270 | */
    EulerA(0,0,1.75*M_PI),		// MAV_SENSOR_ROTATION_YAW_315=7, 			Roll: 0, Pitch: 0, Yaw: 315 | */
    EulerA(M_PI,0,0), 			// MAV_SENSOR_ROTATION_ROLL_180=8,  		Roll: 180, Pitch: 0, Yaw: 0 | */
    EulerA(M_PI,0,0.25*M_PI),	// MAV_SENSOR_ROTATION_ROLL_180_YAW_45=9, 	Roll: 180, Pitch: 0, Yaw: 45 | */
    EulerA(M_PI,0,0.50*M_PI),	// MAV_SENSOR_ROTATION_ROLL_180_YAW_90=10, 	Roll: 180, Pitch: 0, Yaw: 90 | */
    EulerA(M_PI,0,0.75*M_PI),	// MAV_SENSOR_ROTATION_ROLL_180_YAW_135=11, Roll: 180, Pitch: 0, Yaw: 135 | */
    EulerA(0,M_PI,0), 			// MAV_SENSOR_ROTATION_PITCH_180=12, 		Roll: 0, Pitch: 180, Yaw: 0 | */
   	EulerA(M_PI,0,1.25*M_PI),	// MAV_SENSOR_ROTATION_ROLL_180_YAW_225=13, Roll: 180, Pitch: 0, Yaw: 225 | */
    EulerA(M_PI,0,1.50*M_PI),	// MAV_SENSOR_ROTATION_ROLL_180_YAW_270=14, Roll: 180, Pitch: 0, Yaw: 270 | */
    EulerA(M_PI,0,1.75*M_PI),	// MAV_SENSOR_ROTATION_ROLL_180_YAW_315=15, Roll: 180, Pitch: 0, Yaw: 315 | */
    EulerA(M_PI/4,0,0), 		// MAV_SENSOR_ROTATION_ROLL_90=16, 			Roll: 90, Pitch: 0, Yaw: 0 | */
    EulerA(M_PI/4,0,0.25*M_PI),	// MAV_SENSOR_ROTATION_ROLL_90_YAW_45=17, 	Roll: 90, Pitch: 0, Yaw: 45 | */
    EulerA(M_PI/4,0,0.50*M_PI), 	// MAV_SENSOR_ROTATION_ROLL_90_YAW_90=18, 	Roll: 90, Pitch: 0, Yaw: 90 | */
    EulerA(M_PI/4,0,0.75*M_PI), 	// MAV_SENSOR_ROTATION_ROLL_90_YAW_135=19, 	Roll: 90, Pitch: 0, Yaw: 135 | */
    EulerA(3*M_PI/2,0,0), 		// MAV_SENSOR_ROTATION_ROLL_270=20,			Roll: 270, Pitch: 0, Yaw: 0 | */
    EulerA(3*M_PI/2,0,0.25*M_PI),// MAV_SENSOR_ROTATION_ROLL_270_YAW_45=21, 	Roll: 270, Pitch: 0, Yaw: 45 | */
    EulerA(3*M_PI/2,0,0.50*M_PI),// MAV_SENSOR_ROTATION_ROLL_270_YAW_90=22, 	Roll: 270, Pitch: 0, Yaw: 90 | */
    EulerA(3*M_PI/2,0,0.75*M_PI),// MAV_SENSOR_ROTATION_ROLL_270_YAW_135=23, Roll: 270, Pitch: 0, Yaw: 135 | */
    EulerA(0,M_PI/2,0), 		// MAV_SENSOR_ROTATION_PITCH_90=24, 		Roll: 0, Pitch: 90, Yaw: 0 | */
    EulerA(0,3*M_PI/2,0), 		// MAV_SENSOR_ROTATION_PITCH_270=25, 		Roll: 0, Pitch: 270, Yaw: 0 | */
    EulerA(0,M_PI,M_PI/2), 		// MAV_SENSOR_ROTATION_PITCH_180_YAW_90=26  Roll: 0, Pitch: 180, Yaw: 90 | */
    EulerA(0,M_PI,3*M_PI/2), 	// MAV_SENSOR_ROTATION_PITCH_180_YAW_270=27 Roll: 0, Pitch: 180, Yaw: 270 | */
    EulerA(M_PI/2,M_PI/2,0), 	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_90=28  Roll: 90, Pitch: 90, Yaw: 0 | */
    EulerA(M_PI,M_PI/2,0), 		// MAV_SENSOR_ROTATION_ROLL_180_PITCH_90=29 Roll: 180, Pitch: 90, Yaw: 0 | */
    EulerA(3*M_PI/2,M_PI/2,0), 	// MAV_SENSOR_ROTATION_ROLL_270_PITCH_90=30 Roll: 270, Pitch: 90, Yaw: 0 | */
    EulerA(M_PI/2,M_PI,0), 		// MAV_SENSOR_ROTATION_ROLL_90_PITCH_180=31 Roll: 90, Pitch: 180, Yaw: 0 | */
    EulerA(3*M_PI/2,M_PI,0),	// MAV_SENSOR_ROTATION_ROLL_270_PITCH_180=32 Roll: 270, Pitch: 180, Yaw: 0 | */
    EulerA(M_PI/2,3*M_PI/2,0), 	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_270=33  Roll: 90, Pitch: 270, Yaw: 0 | */
    EulerA(M_PI,3*M_PI/2,0),			// MAV_SENSOR_ROTATION_ROLL_180_PITCH_270=34 Roll: 180, Pitch: 270, Yaw: 0 | */
    EulerA(3*M_PI/2,1.50*M_PI,0),// MAV_SENSOR_ROTATION_ROLL_270_PITCH_270=35 Roll: 270, Pitch: 270, Yaw: 0 | */
    EulerA(M_PI/2,M_PI,0.50*M_PI), // MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90=36 Roll: 90, Pitch: 180, Yaw: 90 | */
    EulerA(M_PI/2,0,1.50*M_PI), 	// MAV_SENSOR_ROTATION_ROLL_90_YAW_270=37 	 Roll: 90, Pitch: 0, Yaw: 270 | */
    EulerA(1.75*M_PI,1.75*M_PI,1.75*M_PI)	// MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315=38 Roll: 315, Pitch: 315, Yaw: 315 | */
};

matrix::Dcm<float> sensorOrientationToAttitude(uint8_t orientation)
{
	EulerA rot = rotations[orientation];
	return matrix::Dcm<float>(rotations[orientation]);
};
