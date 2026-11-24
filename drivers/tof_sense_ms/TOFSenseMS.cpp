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

#include <cmath>
#include <fcntl.h>
#include <unistd.h>

#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>

namespace
{

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

constexpr float kPointFovRad = 0.040f;     // ~2.3 deg, single point series
constexpr float kPixelFovRad = 1.100f;     // ~63 deg, TOFSense-M / MS 8x8 zones
constexpr float kPointMinDistance = 0.30f; // single point series blind zone

} // namespace

TOFSenseMS::TOFSenseMS(const char *port, uint32_t baud, uint8_t rotation, uint8_t aggregation,
		       float min_distance, float max_distance) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_baud(baud),
	_aggregation(aggregation),
	_min_distance(min_distance),
	_max_distance(max_distance)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_TOFMS;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	// '/dev/ttySx' -> x
	const int bus_num = atoi(&_port[strlen(_port) - 1]);

	if (bus_num >= 0 && bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

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
	perf_free(_invalid_frames);
}

int TOFSenseMS::configure_uart()
{
	termios uart_config {};

	if (tcgetattr(_fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr failed (%d)", errno);
		return PX4_ERROR;
	}

	const speed_t speed = baud_to_speed(_baud);

	if (speed == 0) {
		PX4_ERR("unsupported baudrate %" PRIu32, _baud);
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
		PX4_ERR("failed to set baudrate %" PRIu32, _baud);
		return PX4_ERROR;
	}

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed (%d)", errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int TOFSenseMS::init()
{
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open %s failed (%d)", _port, errno);
		return PX4_ERROR;
	}

	if (configure_uart() != PX4_OK) {
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	// drain anything already buffered before we start
	uint8_t discard[64];

	while (::read(_fd, discard, sizeof(discard)) > 0) {}

	_parser.reset();
	_last_frame_count = 0;
	_last_frame_time = hrt_absolute_time();

	start();

	return PX4_OK;
}

void TOFSenseMS::start()
{
	// the sensor streams continuously; poll a bit faster than the frame rate
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

	uint8_t buffer[128];
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

	perf_end(_sample_perf);
}

bool TOFSenseMS::aggregate(const tof_sense_ms::Frame &frame, float &distance, int8_t &quality)
{
	float distances[tof_sense_ms::kPixelMax];
	uint32_t signals[tof_sense_ms::kPixelMax];
	int count{0};
	int center_count{0};
	float center_sum{0.f};
	uint32_t center_signal_sum{0};

	const int zones = frame.pixel_count;

	// square grid (8x8 or 4x4) -> use the center quarter for the distance
	const int side = static_cast<int>(sqrtf(static_cast<float>(zones)));
	const bool square = (side * side == zones);
	const int center_lo = square ? side / 4 : 0;
	const int center_hi = square ? (side - 1 - side / 4) : -1;

	for (int i = 0; i < zones && i < tof_sense_ms::kPixelMax; i++) {
		const tof_sense_ms::Pixel &pixel = frame.pixels[i];

		if (pixel.status != 0) {
			continue;
		}

		const float d = pixel.distance_mm * 0.001f;

		if (!PX4_ISFINITE(d) || d < _min_distance || d > _max_distance) {
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

	if (count == 0) {
		return false;
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
		// median (also the fallback for AGGREGATION_CENTER)
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

	// signal strength is 16 bit; the M series reports roughly 0..200
	const float mean_signal = static_cast<float>(signal_sum) / math::max(used, 1);
	quality = static_cast<int8_t>(math::constrain(static_cast<int>(mean_signal), 0, 100));

	return true;
}

void TOFSenseMS::handle_frame(const hrt_abstime &timestamp)
{
	// adapt FoV / range to the series that is actually connected
	if (_frame.type != _last_frame_type) {
		_last_frame_type = _frame.type;

		if (_frame.type == 0) {
			_px4_rangefinder.set_min_distance(kPointMinDistance);
			_px4_rangefinder.set_fov(kPointFovRad);

		} else {
			_px4_rangefinder.set_min_distance(_min_distance);
			_px4_rangefinder.set_fov(kPixelFovRad);
		}
	}

	// frame rate, updated once per second
	const uint32_t frame_count = _parser.frames();

	if (timestamp - _last_frame_time > 1_s) {
		const float elapsed = (timestamp - _last_frame_time) * 1e-6f;
		_frame_rate = static_cast<uint32_t>((frame_count - _last_frame_count) / math::max(elapsed, 1e-3f));
		_last_frame_time = timestamp;
		_last_frame_count = frame_count;
	}

	float distance{0.f};
	int8_t quality{-1};

	if (aggregate(_frame, distance, quality)) {
		_last_distance = distance;
		_px4_rangefinder.update(timestamp, distance, quality);

	} else {
		perf_count(_invalid_frames);
	}
}

void TOFSenseMS::print_info()
{
	PX4_INFO("port: %s, baud: %" PRIu32 ", aggregation: %s", _port, _baud,
		 _aggregation == AGGREGATION_CENTER ? "center" :
		 (_aggregation == AGGREGATION_MEDIAN ? "median" : "mean"));
	PX4_INFO("frames: %" PRIu32 ", frame rate: %" PRIu32 " Hz, parser errors: %" PRIu32
		 " (checksum: %" PRIu32 ")",
		 _parser.frames(), _frame_rate, _parser.errors(), _parser.checksum_errors());
	PX4_INFO("last frame: type %u, zones %u, distance %.3f m", static_cast<unsigned>(_frame.type),
		 static_cast<unsigned>(_frame.pixel_count), (double)_last_distance);

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_invalid_frames);
}
