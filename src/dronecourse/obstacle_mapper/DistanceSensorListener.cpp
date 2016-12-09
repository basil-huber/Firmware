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

#include "DistanceSensorListener.hpp"
#include <iostream>

DistanceSensorListener::DistanceSensorListener() :
	_attitude_subscription(ORB_ID(vehicle_attitude), SUBSCRIB_INTERVAL, 0),
	_local_pos_subscription(ORB_ID(vehicle_local_position), SUBSCRIB_INTERVAL, 0),
	_subscriptions({
		uORB::Subscription<distance_sensor_s>(ORB_ID(distance_sensor), SUBSCRIB_INTERVAL, 0),
		uORB::Subscription<distance_sensor_s>(ORB_ID(distance_sensor), SUBSCRIB_INTERVAL, 1),
		uORB::Subscription<distance_sensor_s>(ORB_ID(distance_sensor), SUBSCRIB_INTERVAL, 2),
		uORB::Subscription<distance_sensor_s>(ORB_ID(distance_sensor), SUBSCRIB_INTERVAL, 3)}),
	_sensorCount(0)
{
	// set timestamps to 0
	for(uint8_t i = 0; i < N_SUBS; i++)
	{
		_timestamps[i] = 0;
	}
}


bool DistanceSensorListener::update()
{
	bool isUpdated = false;

	_attitude_subscription.update();
	_local_pos_subscription.update();


	// iterate over list of all uORB subscription and update each; check for newly connected sensors
	for(uint8_t i = 0; i < N_SUBS; i++)
	{
		uORB::Subscription<distance_sensor_s>* sub = &_subscriptions[i];

			sub->update();
			distance_sensor_s msg = sub->get();
			if(msg.timestamp != _timestamps[i])
			{
				_timestamps[i] = msg.timestamp;
				isUpdated = true;
				
				// look for sensor with corresponding id
				DistanceSensor* sensor = getSensor(msg.id);
				
				// if sensor does not exist, create (initialize) one
				if(sensor == NULL)
				{
					sensor = addSensor(msg.id, msg.orientation, msg.type);
					if(sensor != NULL)
					{
						std::cout << "ADDED NEW SENSOR from subs[" << (int)i << "]; id = " << (int)msg.id << std::endl;
					}
				}

				if(sensor != NULL)
				{
					// set measurements
					//std::cout << "SUB " << (int)i << "  id " << (int)msg.id << "   : " << msg.current_distance << std::endl;
					// get current attitude as Direct Cosine Matrix (Dcm)
					vehicle_attitude_s att_msg = _attitude_subscription.get();
					matrix::Dcm<float> att(matrix::Quaternion<float>(att_msg.q));
					// get current local position as Vector<float,3>
					vehicle_local_position_s local_pos_msg = _local_pos_subscription.get();
					matrix::Vector<float,3> local_pos;
					local_pos(0) = local_pos_msg.x;
					local_pos(1) = local_pos_msg.y;
					local_pos(2) = local_pos_msg.z;
					sensor->setMeasurement(msg.min_distance,
											msg.max_distance,
											msg.current_distance,
											msg.covariance,
											msg.timestamp,
											att,
											local_pos);
					Position obstacle_position_lf = sensor->getObstaclePositionLf();
					std::cout << "Obstacle sensed (sensor " << (int)sensor->getId() << ") at " << obstacle_position_lf(0) << "," << obstacle_position_lf(1) << "," << obstacle_position_lf(2) << std::endl;
				}
				else
				{
					std::cout << "NO SLOTS FOR NEW SENSOR AVAILABLE for sensor " << (int)msg.id << std::endl;
				}
			}
	}
	return isUpdated;
}


DistanceSensor* DistanceSensorListener::getSensor(uint8_t id)
{
	DistanceSensor* sensor = NULL;
	for(uint8_t i = 0; i < _sensorCount; i++)
	{
		if(_sensors[i]->getId() == id)
		{
			sensor = _sensors[i];
			break;
		}
	}

	return sensor;
}


DistanceSensor* DistanceSensorListener::addSensor(uint8_t id, uint8_t orientation, uint8_t type)
{
	// check if there is space for an additional sensor
	if(_sensorCount >= N_SENS_MAX)
	{
		return NULL;
	}

	DistanceSensor* sensor = NULL;

	// iterate over _sensorSlots to find an inactive sensor
	for(uint8_t i = 0; i < N_SENS_MAX; i++)
	{
		if(!_sensorSlots[i].isInitialized())
		{
			sensor = &_sensorSlots[i];
			sensor->initialize(id, orientation, type);
			_sensors[_sensorCount] = sensor;
			_sensorCount++;
			break;
		}
	}

	return sensor;
}