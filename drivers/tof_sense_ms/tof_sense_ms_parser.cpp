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
 * @file tof_sense_ms_parser.cpp
 */

#include "tof_sense_ms_parser.h"

namespace tof_sense_ms
{

namespace
{

inline uint16_t read_u16(const uint8_t *buffer)
{
	return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
}

inline uint32_t read_u24(const uint8_t *buffer)
{
	return static_cast<uint32_t>(buffer[0])
	       | (static_cast<uint32_t>(buffer[1]) << 8)
	       | (static_cast<uint32_t>(buffer[2]) << 16);
}

inline uint32_t read_u32(const uint8_t *buffer)
{
	return static_cast<uint32_t>(buffer[0])
	       | (static_cast<uint32_t>(buffer[1]) << 8)
	       | (static_cast<uint32_t>(buffer[2]) << 16)
	       | (static_cast<uint32_t>(buffer[3]) << 24);
}

} // namespace

void Parser::reset()
{
	_state = State::Sync;
	_length = 0;
}

void Parser::restart(uint8_t byte)
{
	reset();

	if (byte == kFrameHeader) {
		_buffer[0] = byte;
		_length = 1;
		_state = State::Function;
	}
}

bool Parser::checksum_ok(uint16_t length) const
{
	if (length < 2 || length > kFrameMaxLength) {
		return false;
	}

	uint8_t checksum = 0;

	for (uint16_t i = 0; i < length - 1; i++) {
		checksum = static_cast<uint8_t>(checksum + _buffer[i]);
	}

	return checksum == _buffer[length - 1];
}

bool Parser::push(uint8_t byte, Frame &frame)
{
	switch (_state) {
	case State::Sync:
		if (byte == kFrameHeader) {
			_buffer[0] = byte;
			_length = 1;
			_state = State::Function;
		}

		break;

	case State::Function:
		_buffer[_length++] = byte;

		if (byte == kFrameFunctionPoint) {
			_type = 0;
			_length_min = kPointFrameLength;
			_length_max = kPointFrameLength;
			_state = State::Payload;

		} else if (byte == kFrameFunctionPixel) {
			_type = 1;
			_state = State::Header;

		} else {
			_errors++;
			restart(byte);
		}

		break;

	case State::Header:
		_buffer[_length++] = byte;

		if (_length < 9) {
			break;
		}

		{
			const uint8_t zones = _buffer[8];

			if (zones == 0 || zones > kPixelMax) {
				_errors++;
				restart(byte);
				break;
			}

			// 9 byte header + zones * 6 byte + checksum (+ 6 byte trailer seen on the M series)
			_length_min = static_cast<uint16_t>(9 + 6 * zones + 1);
			_length_max = static_cast<uint16_t>(_length_min + 6);
			_state = State::Payload;
		}

		break;

	case State::Payload:
		_buffer[_length++] = byte;

		if (_length < _length_min) {
			break;
		}

		if (checksum_ok(_length)) {
			return finalize(frame);
		}

		if (_length >= _length_max) {
			// no valid frame: drop it and try to resync on the next header
			_checksum_errors++;
			_errors++;
			restart(byte);
		}

		break;
	}

	return false;
}

bool Parser::finalize(Frame &frame)
{

	frame.type = _type;
	frame.id = _buffer[3];
	frame.system_time_ms = read_u32(&_buffer[4]);

	if (_type == 0) {
		frame.pixel_count = 1;

		int32_t distance = static_cast<int32_t>(read_u24(&_buffer[8]));

		// sign extend the 24 bit value
		if (distance & 0x800000) {
			distance |= static_cast<int32_t>(0xFF000000);
		}

		frame.pixels[0].distance_mm = distance;
		frame.pixels[0].status = _buffer[11];
		frame.pixels[0].signal = read_u16(&_buffer[12]);

	} else {
		const uint8_t zones = _buffer[8];
		frame.pixel_count = zones;

		for (uint8_t i = 0; i < zones; i++) {
			const uint8_t *pixel = &_buffer[9 + 6 * i];
			frame.pixels[i].distance_mm = static_cast<int32_t>(read_u24(pixel) / 1000);  // um -> mm
			frame.pixels[i].status = pixel[3];
			frame.pixels[i].signal = read_u16(&pixel[4]);
		}
	}

	_frames++;
	reset();

	return true;
}

} // namespace tof_sense_ms
