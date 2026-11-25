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
 * @file TOFSenseMS.cpp
 */

#include "TOFSenseMS.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>

namespace
{

constexpr float kPointFovRad = 0.040f;      // ~2.3 deg, single point series
constexpr float kPixelFovRad = 1.100f;      // ~63 deg, TOFSense-M / MS zone map
constexpr float kPointMinDistance = 0.30f;  // single point series blind zone
constexpr hrt_abstime kLinkTimeout{1_s};    // no valid frame -> count a timeout

speed_t baud_to_speed(uint32_t baud)
{
	switch (baud) {
	case 9600:    return B9600;
	case 19200:   return B19200;
	case 38400:   return B38400;
	case 57600:   return B57600;
	case 115200:  return B115200;
	case 230400:  return B230400;
	case 460800:  return B460800;
	case 921600:  return B921600;
	case 1000000: return B1000000;
	case 1500000: return B1500000;
	case 2000000: return B2000000;
	case 3000000: return B3000000;
	default:      return 0;
	}
}

/** Extract ttySx from the device path for the device id bus field. */
uint8_t serial_port_bus(const char *port)
{
	uint32_t value{0};
	bool found{false};

	for (const char *p = port; *p != '\0'; p++) {
		if (*p >= '0' && *p <= '9') {
			value = value * 10 + static_cast<uint32_t>(*p - '0');
			found = true;

			if (value > 31) {
				// The device id bus field is only 5 bits wide.
				return 0;
			}
		}
	}

	return found ? static_cast<uint8_t>(value) : 0;
}

const char *aggregation_name(uint8_t aggregation)
{
	switch (aggregation) {
	case TOFSenseMS::AGGREGATION_CENTER: return "center";
	case TOFSenseMS::AGGREGATION_MEDIAN: return "median";
	case TOFSenseMS::AGGREGATION_MEAN:   return "mean";
	default:                             return "unknown";
	}
}

} // namespace

TOFSenseMS::TOFSenseMS(uint8_t instance, const char *port, uint32_t baud, uint8_t rotation,
		       uint8_t aggregation, float min_distance, float max_distance) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_instance(instance),
	_baud(baud),
	_rotation(rotation),
	_aggregation(aggregation),
	_min_distance(min_distance),
	_max_distance(max_distance)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	snprintf(_perf_sample_name, sizeof(_perf_sample_name), MODULE_NAME ":%u read", instance);
	snprintf(_perf_comms_name, sizeof(_perf_comms_name), MODULE_NAME ":%u com_err", instance);
	_sample_perf = perf_alloc(PC_ELAPSED, _perf_sample_name);
	_comms_errors = perf_alloc(PC_COUNT, _perf_comms_name);

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_TOFMS;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = serial_port_bus(_port);
	device_id.devid_s.address = 0;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_px4_rangefinder.set_min_distance(_min_distance);
	_px4_rangefinder.set_max_distance(_max_distance);
	_px4_rangefinder.set_fov(kPixelFovRad);
}

TOFSenseMS::~TOFSenseMS()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TOFSenseMS::configure_uart()
{
	termios uart_config {};

	if (tcgetattr(_fd, &uart_config) < 0) {
		PX4_ERR("%s: tcgetattr failed (%d)", _port, errno);
		return PX4_ERROR;
	}

	const speed_t speed = baud_to_speed(_baud);

	if (speed == 0) {
		PX4_ERR("%s: unsupported baudrate %" PRIu32, _port, _baud);
		return PX4_ERROR;
	}

	// raw 8N1, no flow control
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~CRTSCTS;

	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config.c_oflag &= ~(OPOST | ONLCR);

	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("%s: failed to set baudrate %" PRIu32, _port, _baud);
		return PX4_ERROR;
	}

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("%s: tcsetattr failed (%d)", _port, errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int TOFSenseMS::init()
{
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("instance %u: open %s failed (%d)", _instance, _port, errno);
		return PX4_ERROR;
	}

	if (configure_uart() != PX4_OK) {
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	// Drain anything already buffered before starting a fresh parse.
	uint8_t discard[64];

	while (::read(_fd, discard, sizeof(discard)) > 0) {}

	_parser.reset();

	const hrt_abstime now = hrt_absolute_time();
	_last_valid_frame = now;
	_last_rate_time = now;
	_last_rate_frames = 0;

	start();

	return PX4_OK;
}

void TOFSenseMS::start()
{
	// Poll a bit faster than the sensor frame rate (default 10 Hz).
	ScheduleOnInterval(4_ms);
}

void TOFSenseMS::stop()
{
	ScheduleClear();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

void TOFSenseMS::Run()
{
	if (_fd < 0) {
		return;
	}

	perf_begin(_sample_perf);

	uint8_t buffer[256];
	int bytes_read{0};
	int iterations{0};

	while (iterations++ < 16 && (bytes_read = ::read(_fd, buffer, sizeof(buffer))) > 0) {
		for (int i = 0; i < bytes_read; i++) {
			if (_parser.push(buffer[i], _frame)) {
				handle_frame(hrt_absolute_time());
			}
		}
	}

	if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
		perf_count(_comms_errors);
	}

	check_link_timeout(hrt_absolute_time());

	perf_end(_sample_perf);
}

void TOFSenseMS::check_link_timeout(const hrt_abstime &timestamp)
{
	if (_last_valid_frame == 0) {
		_last_valid_frame = timestamp;
		return;
	}

	if ((timestamp - _last_valid_frame) > kLinkTimeout) {
		if (!_timeout_active) {
			_timeout_active = true;
			_timeout_events++;

			PX4_WARN("instance %u: no valid frame on %s for %" PRIu32 " ms (check baudrate and wiring)",
				 _instance, _port, static_cast<uint32_t>((timestamp - _last_valid_frame) / 1000));

			// A stale partial frame must not be prepended to the next valid frame.
			_parser.reset();
		}
	}
}

TOFSenseMS::AggregateResult TOFSenseMS::aggregate(const tof_sense_ms::Frame &frame,
		float &distance, int8_t &quality, int &used_pixels)
{
	float distances[tof_sense_ms::kPixelMax];
	uint32_t signals[tof_sense_ms::kPixelMax];
	int count{0};
	int center_count{0};
	float center_sum{0.f};
	uint32_t center_signal_sum{0};
	int valid_status_count{0};

	const int zones = frame.pixel_count;

	// The M/MS series supports 8x8 (64 zones) and 4x4 (16 zones). Use the
	// center quarter for altitude hold and fall back to the median when the
	// center is blocked.
	const int side = static_cast<int>(std::sqrt(static_cast<float>(zones)));
	const bool square = (side * side == zones);
	const int center_lo = square ? side / 4 : 0;
	const int center_hi = square ? (side - 1 - side / 4) : -1;

	const float min_distance = (frame.type == 0) ? math::max(_min_distance, kPointMinDistance) : _min_distance;

	for (int i = 0; i < zones && i < tof_sense_ms::kPixelMax; i++) {
		const tof_sense_ms::Pixel &pixel = frame.pixels[i];

		if (pixel.status != 0) {
			continue;
		}

		valid_status_count++;

		const float d = pixel.distance_mm * 0.001f;

		if (!PX4_ISFINITE(d) || d < min_distance || d > _max_distance) {
			continue;
		}

		distances[count] = d;
		signals[count] = pixel.signal;
		count++;

		if (square) {
			const int row = i / side;
			const int col = i % side;

			if (row >= center_lo && row <= center_hi && col >= center_lo && col <= center_hi) {
				center_sum += d;
				center_signal_sum += pixel.signal;
				center_count++;
			}
		}
	}

	if (valid_status_count == 0) {
		return AggregateResult::NoValidPixels;
	}

	if (count == 0) {
		return AggregateResult::OutOfRange;
	}

	float result{0.f};
	uint32_t signal_sum{0};
	int used{0};

	if ((_aggregation == AGGREGATION_CENTER) && (center_count > 0)) {
		result = center_sum / center_count;
		signal_sum = center_signal_sum;
		used = center_count;

	} else if (_aggregation == AGGREGATION_MEAN) {
		for (int i = 0; i < count; i++) {
			result += distances[i];
			signal_sum += signals[i];
		}

		result /= count;
		used = count;

	} else {
		// Median, also the fallback when the center zones are invalid.
		for (int i = 1; i < count; i++) {   // insertion sort, count <= 64
			const float key = distances[i];
			const uint32_t key_signal = signals[i];
			int j = i - 1;

			while (j >= 0 && distances[j] > key) {
				distances[j + 1] = distances[j];
				signals[j + 1] = signals[j];
				j--;
			}

			distances[j + 1] = key;
			signals[j + 1] = key_signal;
		}

		result = distances[count / 2];
		signal_sum = signals[count / 2];
		used = 1;
	}

	distance = result;
	used_pixels = used;

	// Signal strength is 16 bit; the M/MS series reports roughly 0..200.
	const float mean_signal = static_cast<float>(signal_sum) / math::max(used, 1);
	quality = static_cast<int8_t>(math::constrain(static_cast<int>(mean_signal), 0, 100));

	return AggregateResult::Ok;
}

void TOFSenseMS::handle_frame(const hrt_abstime &timestamp)
{
	_timeout_active = false;
	_last_valid_frame = timestamp;

	const uint32_t frame_count = _parser.frames();

	if (timestamp - _last_rate_time >= 1_s) {
		const float elapsed = (timestamp - _last_rate_time) * 1e-6f;
		_frame_rate = static_cast<uint32_t>((frame_count - _last_rate_frames) / math::max(elapsed, 1e-3f));
		_last_rate_time = timestamp;
		_last_rate_frames = frame_count;
	}

	// Adapt FoV and minimum distance to the series that is actually connected.
	if (_frame.type != _last_frame_type) {
		_last_frame_type = _frame.type;

		if (_frame.type == 0) {
			_px4_rangefinder.set_min_distance(math::max(_min_distance, kPointMinDistance));
			_px4_rangefinder.set_fov(kPointFovRad);

		} else {
			_px4_rangefinder.set_min_distance(_min_distance);
			_px4_rangefinder.set_fov(kPixelFovRad);
		}
	}

	float distance{0.f};
	int8_t quality{-1};
	int used_pixels{0};

	switch (aggregate(_frame, distance, quality, used_pixels)) {
	case AggregateResult::Ok:
		_last_distance = distance;
		_frames_published++;
		_px4_rangefinder.update(timestamp, distance, quality);
		break;

	case AggregateResult::NoValidPixels:
		_frames_no_valid_zone++;
		break;

	case AggregateResult::OutOfRange:
		_frames_out_of_range++;
		break;
	}
}

void TOFSenseMS::print_info()
{
	const hrt_abstime now = hrt_absolute_time();
	const uint32_t last_frame_age_ms = static_cast<uint32_t>((now - _last_valid_frame) / 1000);

	PX4_INFO("instance %u: %s @ %" PRIu32 " Bd, orientation %u, aggregation %s, range %.2f..%.2f m",
		 _instance, _port, _baud, static_cast<unsigned>(_rotation), aggregation_name(_aggregation),
		 static_cast<double>(_min_distance), static_cast<double>(_max_distance));

	PX4_INFO("  frames: %" PRIu32 " total, %" PRIu32 " Hz, published %" PRIu32
		 ", no-valid-zone %" PRIu32 ", out-of-range %" PRIu32,
		 _parser.frames(), _frame_rate, _frames_published, _frames_no_valid_zone, _frames_out_of_range);

	PX4_INFO("  parser: header/length errors %" PRIu32 ", checksum errors %" PRIu32,
		 _parser.header_errors() + _parser.length_errors(), _parser.checksum_errors());

	PX4_INFO("  link: timeouts %" PRIu32 ", last frame %" PRIu32 " ms ago, last distance %.3f m"
		 " (type %u, %u zones)",
		 _timeout_events, last_frame_age_ms, static_cast<double>(_last_distance),
		 static_cast<unsigned>(_frame.type), static_cast<unsigned>(_frame.pixel_count));

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
