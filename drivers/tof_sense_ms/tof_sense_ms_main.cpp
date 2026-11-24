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
 * NLink (Nooploop TOFSense / M / MS) range finder driver.
 */

#include "TOFSenseMS.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

namespace tof_sense_ms
{

TOFSenseMS *g_dev{nullptr};

int start(const char *port, uint32_t baud, uint8_t rotation, uint8_t aggregation,
	  float min_distance, float max_distance);
int status();
int stop();
int usage();

int
start(const char *port, uint32_t baud, uint8_t rotation, uint8_t aggregation,
      float min_distance, float max_distance)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	g_dev = new TOFSenseMS(port, baud, rotation, aggregation, min_distance, max_distance);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (PX4_OK != g_dev->init()) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

int
stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for the Nooploop TOFSense / TOFSense-M / TOFSense-MS LiDAR range finder.

The sensor streams NLink frames:
  - TOFSense / -F / -S : 16 byte single point frame (function mark 0x00)
  - TOFSense-M / -MS   : 400 byte frame, 8x8 = 64 zones (function mark 0x01)

The driver validates the frame checksum and reduces the 8x8 zone map to a single
distance (center zones by default) published on the `distance_sensor` uORB topic,
so EKF2 can fuse it as a range finder.

Default UART configuration of the sensor is 921600 8N1, output rate is configured
with the Nooploop NAssistant tool.

### Examples

Start the driver on the UART2 RX pin (U11.6 on the MoonPilot board, /dev/ttyS1):
$ tof_sense_ms start -d /dev/ttyS1

Start on the 6 pin GPS port UART4 instead:
$ tof_sense_ms start -d /dev/ttyS3

Use the median of all valid zones instead of the center zones:
$ tof_sense_ms start -d /dev/ttyS1 -a 1

Stop the driver:
$ tof_sense_ms stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tof_sense_ms", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', TOFSENSEMS_DEFAULT_PORT, nullptr, "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', TOFSENSEMS_DEFAULT_BAUD, 9600, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('a', 0, 0, 2, "Aggregation: 0=center zones, 1=median, 2=mean", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('m', 0.02f, 0.0f, 10.0f, "Minimum distance [m]", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('M', 4.0f, 0.1f, 50.0f, "Maximum distance [m]", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");

	return PX4_OK;
}

} // namespace tof_sense_ms

extern "C" __EXPORT int tof_sense_ms_main(int argc, char *argv[])
{
	if (argc < 2) {
		return tof_sense_ms::usage();
	}

	const char *device_path = TOFSENSEMS_DEFAULT_PORT;
	uint32_t baud = TOFSENSEMS_DEFAULT_BAUD;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	uint8_t aggregation = TOFSenseMS::AGGREGATION_CENTER;
	float min_distance = 0.02f;
	float max_distance = 4.0f;

	if (!strcmp(argv[1], "start")) {

		int myoptind = 2;
		const char *myoptarg = nullptr;
		int ch = 0;

		while ((ch = px4_getopt(argc, argv, "d:b:a:R:m:M:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'd':
				device_path = myoptarg;
				break;

			case 'b':
				baud = static_cast<uint32_t>(atoi(myoptarg));
				break;

			case 'a':
				aggregation = static_cast<uint8_t>(atoi(myoptarg));
				break;

			case 'R':
				rotation = static_cast<uint8_t>(atoi(myoptarg));
				break;

			case 'm':
				min_distance = strtof(myoptarg, nullptr);
				break;

			case 'M':
				max_distance = strtof(myoptarg, nullptr);
				break;

			default:
				return tof_sense_ms::usage();
			}
		}

		if (aggregation > TOFSenseMS::AGGREGATION_MEAN) {
			PX4_ERR("invalid aggregation mode %u", static_cast<unsigned>(aggregation));
			return PX4_ERROR;
		}

		return tof_sense_ms::start(device_path, baud, rotation, aggregation, min_distance, max_distance);
	}

	if (!strcmp(argv[1], "stop")) {
		return tof_sense_ms::stop();
	}

	if (!strcmp(argv[1], "status")) {
		return tof_sense_ms::status();
	}

	return tof_sense_ms::usage();
}
