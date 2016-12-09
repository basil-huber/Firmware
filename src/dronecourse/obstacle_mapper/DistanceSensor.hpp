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

#pragma once

#include <stdint.h>
#include "matrix/Quaternion.hpp"

typedef matrix::Vector<float,3> Position;

class DistanceSensor
{
public:

	DistanceSensor();

	void initialize(uint8_t id, uint8_t orientation = 0, uint8_t type = 0);

	void setMeasurement(float min_distance,
						float max_distance,
						float current_distance,
						float covariance,
						uint64_t timestamp,
						matrix::Dcm<float> vehicle_attitude,
						Position vehicle_position_lf);

	uint8_t getId() const {return _id;}

	float getCurrentDistance() const {return _current_distance;}

	const Position &getObstaclePositionLf() const {return _obstacle_position_lf;}

	const Position &getPositionLf() const {return _position_lf;}

	uint64_t getLastUpdate() const  {return _last_update;}

	bool isInitialized() const {return _isInitialized;}

	bool isActive() const {return _isActive;}


private:
	float  	 _min_distance;
	float  	 _max_distance;
	float  	 _current_distance;
	float 	 _covariance;
	uint64_t _last_update;
	uint8_t  _type;
	uint8_t  _orientation;
	uint8_t  _id;
	bool 	 _isInitialized;
	bool 	 _isActive;

	matrix::Dcm<float> _attitude;

	Position _position_bf;
	Position _position_lf;
	Position _obstacle_position_bf;
	Position _obstacle_position_lf;
};