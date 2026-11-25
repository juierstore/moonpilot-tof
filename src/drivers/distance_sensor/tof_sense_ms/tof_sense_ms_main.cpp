/****************************************************************************
 *
 *   Copyright (c) 2025 MoonPilot / PX4 Development Team. All rights reserved.
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
 * @file tof_sense_ms_main.cpp
 *
 * NLink (Nooploop TOFSense / -M / -MS) range finder driver command line.
 *
 * The preferred way to start the driver is the standard PX4 serial config
 * parameter SENS_TOFMS_CFG (Sensors group in QGC): rc.serial starts the
 * driver on the selected logical serial port at boot. The shell interface
 * below is kept for debugging and multi-instance setups.
 */

#include "TOFSenseMS.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <lib/conversion/rotation.h>

namespace tof_sense_ms
{

TOFSenseMS *g_dev[TOFSENSEMS_MAX_INSTANCES] {};

int start(uint8_t instance, const char *port, uint32_t baud, uint8_t rotation,
	  uint8_t aggregation, float min_distance, float max_distance);
int status(int instance);
int stop(int instance);
int usage();

int start(uint8_t instance, const char *port, uint32_t baud, uint8_t rotation,
	  uint8_t aggregation, float min_distance, float max_distance)
{
	if (instance >= TOFSENSEMS_MAX_INSTANCES) {
		PX4_ERR("invalid instance %u (max %u)", static_cast<unsigned>(instance),
			static_cast<unsigned>(TOFSENSEMS_MAX_INSTANCES - 1));
		return PX4_ERROR;
	}

	if (g_dev[instance] != nullptr) {
		PX4_ERR("instance %u already started on %s", static_cast<unsigned>(instance), g_dev[instance]->port());
		return PX4_OK;
	}

	// Two instances on the same device would fight over the same bytes.
	for (unsigned i = 0; i < TOFSENSEMS_MAX_INSTANCES; i++) {
		if ((i != instance) && (g_dev[i] != nullptr) && (strcmp(g_dev[i]->port(), port) == 0)) {
			PX4_ERR("%s is already used by instance %u", port, i);
			return PX4_ERROR;
		}
	}

	g_dev[instance] = new TOFSenseMS(instance, port, baud, rotation, aggregation, min_distance, max_distance);

	if (g_dev[instance] == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (PX4_OK != g_dev[instance]->init()) {
		PX4_ERR("driver start failed");
		delete g_dev[instance];
		g_dev[instance] = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int status(int instance)
{
	bool found{false};
	int ret{PX4_OK};

	for (unsigned i = 0; i < TOFSENSEMS_MAX_INSTANCES; i++) {
		if ((instance >= 0) && (static_cast<int>(i) != instance)) {
			continue;
		}

		if (g_dev[i] != nullptr) {
			g_dev[i]->print_info();
			found = true;
		}
	}

	if (!found) {
		PX4_ERR("driver not running");
		ret = 1;
	}

	return ret;
}

int stop(int instance)
{
	bool found{false};

	for (unsigned i = 0; i < TOFSENSEMS_MAX_INSTANCES; i++) {
		if ((instance >= 0) && (static_cast<int>(i) != instance)) {
			continue;
		}

		if (g_dev[i] != nullptr) {
			PX4_INFO("stopping instance %u on %s", i, g_dev[i]->port());
			delete g_dev[i];
			g_dev[i] = nullptr;
			found = true;
		}
	}

	if (!found) {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for the Nooploop TOFSense / TOFSense-M / TOFSense-MS LiDAR
range finder (NLink protocol).

The sensor streams NLink frames:
  - TOFSense / -F / -S : 16 byte single point frame (function mark 0x00)
  - TOFSense-M / -MS   : 400 byte frame, 8x8 = 64 zones (function mark 0x01)

The driver validates the frame checksum, filters zones whose status is not
zero and reduces the zone map to a single distance (center zones by default)
published on the `distance_sensor` uORB topic, so EKF2 can fuse it as a range
finder.

Normally the driver is started automatically from the standard PX4 serial
configuration parameter SENS_TOFMS_CFG (QGC: Sensors). A second instance can
be configured with SENS_TOFMS_CFG2, for example a forward looking sensor for
obstacle distance while the first sensor measures altitude.

Default UART configuration of the sensor is 921600 8N1; the output rate is
configured with the Nooploop NAssistant tool.

### Examples

Start instance 0 on TELEM 3, downward facing (the serial_config default):
$ tof_sense_ms start -i 0 -d /dev/ttyS4 -b 921600 -R 25 -a 0

Start a second, forward facing instance for obstacle distance:
$ tof_sense_ms start -i 1 -d /dev/ttyS7 -b 921600 -R 0 -a 1

Use the median of all valid zones instead of the center zones:
$ tof_sense_ms start -d /dev/ttyS4 -a 1

Show statistics of all instances:
$ tof_sense_ms status

Stop all instances:
$ tof_sense_ms stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tof_sense_ms", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, TOFSENSEMS_MAX_INSTANCES - 1, "Driver instance", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', TOFSENSEMS_DEFAULT_PORT, nullptr, "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', TOFSENSEMS_DEFAULT_BAUD, 9600, 3000000,
				      "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_INT('a', 0, 0, 2, "Aggregation: 0=center zones, 1=median, 2=mean", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', distance_sensor_s::ROTATION_DOWNWARD_FACING, 0, ROTATION_MAX - 1,
				     "Sensor orientation (25 = downward facing)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('m', 0.15f, 0.0f, 10.0f, "Minimum distance [m]", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('M', 4.0f, 0.1f, 50.0f, "Maximum distance [m]", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
	PRINT_MODULE_USAGE_PARAM_INT('i', -1, -1, TOFSENSEMS_MAX_INSTANCES - 1, "Driver instance (-1 = all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', -1, -1, TOFSENSEMS_MAX_INSTANCES - 1, "Driver instance (-1 = all)", true);

	return PX4_OK;
}

} // namespace tof_sense_ms

extern "C" __EXPORT int tof_sense_ms_main(int argc, char *argv[])
{
	if (argc < 2) {
		return tof_sense_ms::usage();
	}

	const char *device_path = TOFSENSEMS_DEFAULT_PORT;
	int baud = TOFSENSEMS_DEFAULT_BAUD;
	int rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	int aggregation = TOFSenseMS::AGGREGATION_CENTER;
	float min_distance = 0.15f;
	float max_distance = 4.0f;
	int instance = 0;

	if (!strcmp(argv[1], "start")) {

		int myoptind = 2;
		const char *myoptarg = nullptr;
		int ch = 0;

		while ((ch = px4_getopt(argc, argv, "d:b:a:R:m:M:i:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'd':
				device_path = myoptarg;
				break;

			case 'b':
				if (px4_get_parameter_value(myoptarg, baud) != 0) {
					PX4_ERR("baudrate parsing failed");
					return PX4_ERROR;
				}

				break;

			case 'a':
				if (px4_get_parameter_value(myoptarg, aggregation) != 0) {
					PX4_ERR("aggregation parsing failed");
					return PX4_ERROR;
				}

				break;

			case 'R':
				if (px4_get_parameter_value(myoptarg, rotation) != 0) {
					PX4_ERR("orientation parsing failed");
					return PX4_ERROR;
				}

				break;

			case 'm':
				if (px4_get_parameter_value(myoptarg, min_distance) != 0) {
					PX4_ERR("minimum distance parsing failed");
					return PX4_ERROR;
				}

				break;

			case 'M':
				if (px4_get_parameter_value(myoptarg, max_distance) != 0) {
					PX4_ERR("maximum distance parsing failed");
					return PX4_ERROR;
				}

				break;

			case 'i':
				if (px4_get_parameter_value(myoptarg, instance) != 0) {
					PX4_ERR("instance parsing failed");
					return PX4_ERROR;
				}

				break;

			default:
				return tof_sense_ms::usage();
			}
		}

		if (instance < 0 || instance >= TOFSENSEMS_MAX_INSTANCES) {
			PX4_ERR("invalid instance %d (0..%d)", instance, TOFSENSEMS_MAX_INSTANCES - 1);
			return PX4_ERROR;
		}

		if (aggregation < 0 || aggregation > TOFSenseMS::AGGREGATION_MEAN) {
			PX4_ERR("invalid aggregation mode %d", aggregation);
			return PX4_ERROR;
		}

		if (rotation < 0 || rotation >= ROTATION_MAX) {
			PX4_ERR("invalid orientation %d", rotation);
			return PX4_ERROR;
		}

		if (min_distance < 0.f || max_distance <= min_distance) {
			PX4_ERR("invalid range %.3f..%.3f m", static_cast<double>(min_distance),
				static_cast<double>(max_distance));
			return PX4_ERROR;
		}

		return tof_sense_ms::start(static_cast<uint8_t>(instance), device_path,
					   static_cast<uint32_t>(baud), static_cast<uint8_t>(rotation),
					   static_cast<uint8_t>(aggregation), min_distance, max_distance);
	}

	if (!strcmp(argv[1], "stop") || !strcmp(argv[1], "status")) {
		const bool is_status = !strcmp(argv[1], "status");
		int myoptind = 2;
		const char *myoptarg = nullptr;
		int ch = 0;
		instance = -1;

		while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'i':
				if (px4_get_parameter_value(myoptarg, instance) != 0) {
					PX4_ERR("instance parsing failed");
					return PX4_ERROR;
				}

				break;

			default:
				return tof_sense_ms::usage();
			}
		}

		if (instance < -1 || instance >= TOFSENSEMS_MAX_INSTANCES) {
			PX4_ERR("invalid instance %d", instance);
			return PX4_ERROR;
		}

		return is_status ? tof_sense_ms::status(instance) : tof_sense_ms::stop(instance);
	}

	return tof_sense_ms::usage();
}
