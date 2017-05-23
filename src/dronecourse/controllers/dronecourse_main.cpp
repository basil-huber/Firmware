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
 * @file target_landing_main.cpp
 * Class to track a moving platform
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <fcntl.h>
#include <px4_posix.h>


#include "DronecourseHandler.hpp"


static volatile bool thread_should_exit = false;     /**< Deamon exit flag */
static volatile bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */


static DronecourseHandler::DcMode dc_mode = DronecourseHandler::DcMode::IDLE;
static bool dc_mode_auto = false;

static bool new_pos = false;
static float pos_x = 0;
static float pos_y = 0;
static float pos_z = 5;
static bool new_gimbal_rot = false;
static bool new_gimbal_auto = false;
static float pitch = 0;
static float yaw = 0;

#define DT_US 50000

/**
 * Deamon management function.
 */
extern "C" __EXPORT int dronecourse_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int dronecourse_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static int usage(const char *reason);

static int
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}
	fprintf(stderr, "usage: dronecourse {start|stop|status|pos} [-p <additional params>]\n\n");
	return 1;
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int dronecourse_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;

		deamon_task = px4_task_spawn_cmd("dronecourse",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 1,
						 13000,
						 dronecourse_thread_main,
						 NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			PX4_DEBUG("stop");
			thread_should_exit = true;

		} else {
			PX4_WARN("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			switch(dc_mode)
			{
				case DronecourseHandler::DcMode::IDLE:
					PX4_INFO("is running;  mode : IDLE");
					break;
				case DronecourseHandler::DcMode::POS_CTRL:
					PX4_INFO("is running;  mode : POS_CTRL");
					break;
				case DronecourseHandler::DcMode::FOLLOW:
					PX4_INFO("is running;  mode : FOLLOW");
					break;
				case DronecourseHandler::DcMode::MISSION:
					PX4_INFO("is running;  mode : MISSION");
					break;
			}

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "pos"))
	{
		if(argc < 5){
			usage("coordinates missing");
			return 1;
		}
		// setting new position setpoint
		char* end;
		pos_x = strtod(argv[2], &end);
		pos_y = strtod(argv[3], &end);
		pos_z = strtod(argv[4], &end);

		new_pos = true;
		dc_mode = DronecourseHandler::DcMode::POS_CTRL;
		dc_mode_auto = false;
		PX4_INFO("Setting position command to ( %f | %f | %f )", (double)pos_x, (double)pos_y, (double)pos_z);
		return 0;
	} else if (!strcmp(argv[1], "follow"))
	{
		dc_mode = DronecourseHandler::DcMode::FOLLOW;
		dc_mode_auto = false;
		PX4_INFO("Switching to follow mode");
		return 0;
	} else if (!strcmp(argv[1], "auto"))
	{
		dc_mode_auto = true;
		PX4_INFO("Switching to auto_mode");
	}else if (!strcmp(argv[1], "gimbal"))
	{
		if(argc < 3)
		{
			usage("gimbal: arguments missing");
			return 1;
		}

		if(!strcmp(argv[2], "auto"))
		{
			PX4_INFO("Setting gimbal to automatic");
			new_gimbal_auto = true;
			new_gimbal_rot = false;
			return 0;
		}
		else if(argc >= 4)
		{
			char* end;
			pitch = strtod(argv[2], &end);
			yaw = strtod(argv[3], &end);

			new_gimbal_rot = true;
			new_gimbal_auto = false;
			PX4_INFO("Setting gimbal command to pitch: %f    yaw: %f )", (double)pitch, (double)yaw);
			return 0;
		} else
		{
			usage("gimbal: coordinates missing");
			return 0;
		}
	}

	usage("unrecognized command");
	return 1;
}


int dronecourse_thread_main(int argc, char *argv[])
{

	PX4_DEBUG("starting");

	thread_running = true;

	DronecourseHandler handler;

	while (!thread_should_exit)
	{
		if (dc_mode == DronecourseHandler::DcMode::POS_CTRL && new_pos)
		{
			handler.set_position_command(pos_x, pos_y, pos_z);
		}
		if (new_gimbal_rot)
		{
			handler.gimbal().set_command(pitch, yaw);
			new_gimbal_rot = false;
		}else if(new_gimbal_auto){
			handler.gimbal().setAutomatic();
			new_gimbal_auto  = false;
		}
		handler.set_mode(dc_mode);
		if(dc_mode_auto)
		{
			handler.set_mode_auto();
		}
		handler.update();
		usleep(DT_US);
	}

	PX4_DEBUG("exiting.");

	thread_running = false;

	return 0;
}