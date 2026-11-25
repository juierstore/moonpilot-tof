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
 * @file TOFSenseMS.hpp
 *
 * Driver for the Nooploop TOFSense / TOFSense-M / TOFSense-MS laser range
 * finders connected over a serial port (NLink protocol, 921600 8N1 by
 * default).
 *
 * The class itself is board independent: the serial port, mounting
 * orientation and aggregation mode are all passed in through the command
 * line / module parameters.
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/distance_sensor.h>

#include "tof_sense_ms_parser.h"

#define TOFSENSEMS_DEFAULT_PORT       "/dev/ttyS1"
#define TOFSENSEMS_DEFAULT_BAUD       921600
#define TOFSENSEMS_DEFAULT_ROTATION   distance_sensor_s::ROTATION_DOWNWARD_FACING
#define TOFSENSEMS_MAX_INSTANCES      2

using namespace time_literals;

class TOFSenseMS : public px4::ScheduledWorkItem
{
public:
	enum AggregationMode : uint8_t {
		AGGREGATION_CENTER = 0,  ///< mean of the center zones, median fallback
		AGGREGATION_MEDIAN = 1,  ///< median of all valid zones
		AGGREGATION_MEAN   = 2,  ///< mean of all valid zones
	};

	TOFSenseMS(uint8_t instance, const char *port, uint32_t baud, uint8_t rotation,
		   uint8_t aggregation, float min_distance, float max_distance);
	~TOFSenseMS() override;

	int init();

	void print_info();

	uint8_t instance() const { return _instance; }
	const char *port() const { return _port; }

private:
	void Run() override;

	void start();
	void stop();

	int configure_uart();
	void handle_frame(const hrt_abstime &timestamp);
	void check_link_timeout(const hrt_abstime &timestamp);

	enum class AggregateResult : uint8_t {
		Ok,
		NoValidPixels,   ///< every zone is marked invalid by the sensor
		OutOfRange,      ///< zones are valid but outside [min, max]
	};

	/**
	 * Reduce the (up to 64) zones of a frame to a single distance.
	 *
	 * @param distance    [out] distance in metres
	 * @param quality     [out] 0..100, -1 if unknown
	 * @param used_pixels [out] number of zones that contributed
	 */
	AggregateResult aggregate(const tof_sense_ms::Frame &frame, float &distance,
				  int8_t &quality, int &used_pixels);

	PX4Rangefinder _px4_rangefinder;

	tof_sense_ms::Parser _parser;
	tof_sense_ms::Frame _frame;

	const uint8_t _instance;
	char _port[20] {};
	uint32_t _baud;
	const uint8_t _rotation;
	uint8_t _aggregation;
	float _min_distance;
	float _max_distance;
	uint8_t _last_frame_type{0xFF};
	float _last_distance{0.f};

	int _fd{-1};

	hrt_abstime _last_valid_frame{0};
	hrt_abstime _last_rate_time{0};
	uint32_t _last_rate_frames{0};
	uint32_t _frame_rate{0};
	bool _timeout_active{false};

	uint32_t _frames_published{0};
	uint32_t _frames_no_valid_zone{0};
	uint32_t _frames_out_of_range{0};
	uint32_t _timeout_events{0};

	// perf_alloc() keeps the name pointer, so every instance needs its own
	// string storage.
	char _perf_sample_name[32] {};
	char _perf_comms_name[32] {};
	perf_counter_t _sample_perf{nullptr};
	perf_counter_t _comms_errors{nullptr};
};
