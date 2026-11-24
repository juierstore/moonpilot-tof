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
 * @file tof_sense_ms_parser.h
 *
 * NLink UART frame parser for the Nooploop TOFSense range finder family.
 *
 * Supported:
 *   - TOFSense / TOFSense-F / TOFSense-S   : frame function 0x00, 16 byte frame
 *   - TOFSense-M / TOFSense-MS             : frame function 0x01, 400 byte frame
 *                                            (8x8 = 64 zones, 6 byte per zone)
 *
 * Frame layout (little endian):
 *
 *   offset  size  field
 *   0       1     0x57                       frame header
 *   1       1     function mark              0x00 = point, 0x01 = pixel
 *   2       1     0xFF                       fixed
 *   3       1     id
 *   4       4     system_time [ms]
 *   --- point series ---
 *   8       3     distance, signed 24 bit, unit mm
 *   11      1     distance status
 *   12      2     signal strength
 *   14      1     range precision (reserved)
 *   15      1     checksum
 *   --- pixel series (M/MS) ---
 *   8       1     zone map (pixel count, 64 for 8x8, 16 for 4x4)
 *   9+6*i   3     zone i distance, unsigned 24 bit, unit um
 *   12+6*i  1     zone i distance status
 *   13+6*i  2     zone i signal strength
 *   ...          6 byte trailer (0xFF fill), then checksum as last byte
 *
 * The checksum is the lowest byte of the sum of all previous bytes.
 */

#pragma once

#include <cstdint>

namespace tof_sense_ms
{

static constexpr uint8_t kFrameHeader{0x57};
static constexpr uint8_t kFrameFunctionPoint{0x00};
static constexpr uint8_t kFrameFunctionPixel{0x01};

static constexpr uint8_t kPixelMax{64};
static constexpr uint8_t kPointFrameLength{16};
static constexpr uint16_t kFrameMaxLength{400};

struct Pixel {
	int32_t distance_mm{0};   ///< negative values are invalid (point series only)
	uint8_t status{0};        ///< 0 = measurement valid
	uint16_t signal{0};
};

struct Frame {
	uint8_t type{0};          ///< 0 = point series, 1 = pixel series
	uint8_t id{0};
	uint32_t system_time_ms{0};
	uint8_t pixel_count{0};
	Pixel pixels[kPixelMax] {};
};

class Parser
{
public:
	/**
	 * Feed one byte from the serial stream.
	 *
	 * @return true if a complete, checksum verified frame was decoded into @p frame
	 */
	bool push(uint8_t byte, Frame &frame);

	uint32_t frames() const { return _frames; }
	uint32_t errors() const { return _errors; }
	uint32_t checksum_errors() const { return _checksum_errors; }

	void reset();

private:
	enum class State : uint8_t {
		Sync,
		Function,
		Header,
		Payload,
	};

	bool checksum_ok(uint16_t length) const;
	bool finalize(Frame &frame);
	void restart(uint8_t byte);

	State _state{State::Sync};
	uint8_t _buffer[kFrameMaxLength] {};
	uint16_t _length{0};
	uint16_t _length_min{0};
	uint16_t _length_max{0};
	uint8_t _type{0};
	uint32_t _frames{0};
	uint32_t _errors{0};
	uint32_t _checksum_errors{0};
};

} // namespace tof_sense_ms
