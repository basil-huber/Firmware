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
 * @file DistanceSensorListener.hpp
 * Listens to distance_sensor uORB messages and maintains a list of distance sensors
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_attitude.h>
#include "DistanceSensor.hpp"

class DistanceSensorListener
{
public:

	DistanceSensorListener();

	bool update();

	DistanceSensor* getSensor(uint8_t id);

	DistanceSensor* addSensor(uint8_t id, uint8_t orientation, uint8_t type);

private:
	static const uint8_t N_SUBS = 4;	// maximal number of distance sensors subscriptions
	static const uint8_t N_SENS_MAX = 4;	// maximal number of distance sensors
	static const unsigned SUBSCRIB_INTERVAL  = 1000/100;

	uORB::Subscription<vehicle_attitude_s> _attitude_subscription;
	uORB::Subscription<distance_sensor_s> _subscriptions[N_SUBS];
	uint64_t 							  _timestamps[N_SUBS];
	DistanceSensor 						  _sensorSlots[N_SENS_MAX];
	DistanceSensor*						  _sensors[N_SENS_MAX];	
	uint8_t 							  _sensorCount;
};