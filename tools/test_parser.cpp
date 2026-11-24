/**
 * Host side unit test for the TOFSense NLink parser.
 *
 * Build and run on any machine with a C++ compiler:
 *
 *   python3 gen_sample_frame.py            # fetch the reference frame (once)
 *   g++ -std=c++17 -O1 -Wall -Wextra -o test_parser \
 *       test_parser.cpp ../drivers/tof_sense_ms/tof_sense_ms_parser.cpp -I.
 *   ./test_parser
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../drivers/tof_sense_ms/tof_sense_ms_parser.h"
#include "tof_sample_frame.h"

using namespace tof_sense_ms;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                  \
	do {                                              \
		g_checks++;                               \
		if (!(cond)) {                            \
			g_failures++;                     \
			printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
			printf(__VA_ARGS__);              \
			printf("\n");                     \
		}                                         \
	} while (0)

static uint8_t checksum8(const uint8_t *data, size_t length)
{
	uint8_t sum = 0;

	for (size_t i = 0; i < length; i++) {
		sum = static_cast<uint8_t>(sum + data[i]);
	}

	return sum;
}

static void append_u24le(std::vector<uint8_t> &v, uint32_t value)
{
	v.push_back(value & 0xFF);
	v.push_back((value >> 8) & 0xFF);
	v.push_back((value >> 16) & 0xFF);
}

static void append_u16le(std::vector<uint8_t> &v, uint16_t value)
{
	v.push_back(value & 0xFF);
	v.push_back((value >> 8) & 0xFF);
}

static void append_u32le(std::vector<uint8_t> &v, uint32_t value)
{
	v.push_back(value & 0xFF);
	v.push_back((value >> 8) & 0xFF);
	v.push_back((value >> 16) & 0xFF);
	v.push_back((value >> 24) & 0xFF);
}

/* point series frame: 16 bytes */
static std::vector<uint8_t> make_point_frame(int16_t distance_mm, uint8_t status, uint16_t signal)
{
	std::vector<uint8_t> f;
	f.push_back(0x57);
	f.push_back(0x00);
	f.push_back(0xFF);
	f.push_back(0x00);              // id
	append_u32le(f, 36766);         // system time
	append_u24le(f, static_cast<uint32_t>(static_cast<int32_t>(distance_mm)) & 0xFFFFFF);
	f.push_back(status);
	append_u16le(f, signal);
	f.push_back(0x06);              // range precision
	f.push_back(checksum8(f.data(), f.size()));
	return f;
}

/* M/MS series frame: 9 + 6*pixels (+6 trailer) + 1 */
static std::vector<uint8_t> make_pixel_frame(int pixels, bool trailer,
		const std::vector<int32_t> &distances_mm)
{
	std::vector<uint8_t> f;
	f.push_back(0x57);
	f.push_back(0x01);
	f.push_back(0xFF);
	f.push_back(0x00);              // id
	append_u32le(f, 40963);         // system time
	f.push_back(static_cast<uint8_t>(pixels));

	for (int i = 0; i < pixels; i++) {
		append_u24le(f, static_cast<uint32_t>(distances_mm[i]) * 1000u);  // um
		f.push_back(0x00);          // status
		append_u16le(f, static_cast<uint16_t>(100 + i));
	}

	if (trailer) {
		for (int i = 0; i < 6; i++) {
			f.push_back(0xFF);
		}
	}

	f.push_back(checksum8(f.data(), f.size()));
	return f;
}

static bool feed(Parser &parser, Frame &frame, const std::vector<uint8_t> &bytes)
{
	bool got = false;

	for (uint8_t b : bytes) {
		if (parser.push(b, frame)) {
			got = true;
		}
	}

	return got;
}

int main()
{
	printf("TOFSense NLink parser test\n");

	/* 1. reference frame from the Nooploop protocol documentation */
	{
		printf("[1] reference 8x8 frame from the vendor documentation\n");
		std::vector<uint8_t> frame_bytes(kSampleFrame, kSampleFrame + kSampleFrameSize);
		CHECK(frame_bytes.size() == 400, "sample frame is %zu bytes, expected 400", frame_bytes.size());
		CHECK(checksum8(frame_bytes.data(), frame_bytes.size() - 1) == frame_bytes.back(),
		      "vendor frame checksum");

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, frame_bytes), "vendor frame not decoded");
		CHECK(parser.frames() == 1, "frames %u", parser.frames());
		CHECK(parser.errors() == 0, "errors %u", parser.errors());
		CHECK(frame.type == 1, "type %u", frame.type);
		CHECK(frame.pixel_count == 64, "zones %u", frame.pixel_count);
		CHECK(frame.pixels[0].distance_mm == 492, "zone0 %d mm, expected 492", frame.pixels[0].distance_mm);
		CHECK(frame.pixels[1].distance_mm == 214, "zone1 %d mm, expected 214", frame.pixels[1].distance_mm);
		CHECK(frame.pixels[63].distance_mm == 26, "zone63 %d mm, expected 26", frame.pixels[63].distance_mm);
		CHECK(frame.pixels[0].signal == 159, "zone0 signal %u, expected 159", frame.pixels[0].signal);
	}

	/* 2. single point series */
	{
		printf("[2] single point frame\n");
		Parser parser;
		Frame frame;
		const auto bytes = make_point_frame(2221, 0, 3);
		CHECK(feed(parser, frame, bytes), "point frame not decoded");
		CHECK(frame.type == 0, "type %u", frame.type);
		CHECK(frame.pixel_count == 1, "zones %u", frame.pixel_count);
		CHECK(frame.pixels[0].distance_mm == 2221, "distance %d mm", frame.pixels[0].distance_mm);
		CHECK(frame.pixels[0].signal == 3, "signal %u", frame.pixels[0].signal);
	}

	/* 3. negative distance of the point series */
	{
		printf("[3] single point negative distance\n");
		Parser parser;
		Frame frame;
		const auto bytes = make_point_frame(-1044, 0, 3);
		CHECK(feed(parser, frame, bytes), "point frame not decoded");
		CHECK(frame.pixels[0].distance_mm == -1044, "distance %d mm", frame.pixels[0].distance_mm);
	}

	/* 4. synthetic 8x8 frame */
	{
		printf("[4] synthetic 8x8 frame\n");
		std::vector<int32_t> distances;
		for (int i = 0; i < 64; i++) {
			distances.push_back(2000 + i);
		}

		const auto bytes = make_pixel_frame(64, true, distances);
		CHECK(bytes.size() == 400, "frame size %zu", bytes.size());

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes), "8x8 frame not decoded");
		CHECK(frame.pixel_count == 64, "zones %u", frame.pixel_count);
		CHECK(frame.pixels[0].distance_mm == 2000, "zone0 %d", frame.pixels[0].distance_mm);
		CHECK(frame.pixels[63].distance_mm == 2063, "zone63 %d", frame.pixels[63].distance_mm);
	}

	/* 5. 4x4 frame without the 6 byte trailer */
	{
		printf("[5] 4x4 frame, no trailer\n");
		std::vector<int32_t> distances(16, 1500);
		const auto bytes = make_pixel_frame(16, false, distances);
		CHECK(bytes.size() == 106, "frame size %zu", bytes.size());

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes), "4x4 frame (no trailer) not decoded");
		CHECK(frame.pixel_count == 16, "zones %u", frame.pixel_count);
	}

	/* 6. 4x4 frame with the 6 byte trailer */
	{
		printf("[6] 4x4 frame with trailer\n");
		std::vector<int32_t> distances(16, 1500);
		const auto bytes = make_pixel_frame(16, true, distances);
		CHECK(bytes.size() == 112, "frame size %zu", bytes.size());

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes), "4x4 frame (trailer) not decoded");
		CHECK(frame.pixel_count == 16, "zones %u", frame.pixel_count);
	}

	/* 7. corrupted frame must be rejected and the parser must resync */
	{
		printf("[7] corrupted frame + resync\n");
		Parser parser;
		Frame frame;
		std::vector<int32_t> distances(64, 1234);
		auto bytes = make_pixel_frame(64, true, distances);
		bytes[100] ^= 0xFF;             // break the payload

		CHECK(!feed(parser, frame, bytes), "corrupted frame was accepted");
		CHECK(parser.errors() > 0, "no error counted");

		// a valid frame following the garbage must still be decoded
		auto good = make_pixel_frame(64, true, distances);
		CHECK(feed(parser, frame, good), "no resync after a corrupted frame");
		CHECK(parser.frames() == 1, "frames %u after resync", parser.frames());
	}

	/* 8. partial frame must not be reported */
	{
		printf("[8] partial frame\n");
		Parser parser;
		Frame frame;
		std::vector<int32_t> distances(64, 1000);
		auto bytes = make_pixel_frame(64, true, distances);
		bytes.resize(bytes.size() / 2);
		CHECK(!feed(parser, frame, bytes), "partial frame was accepted");
	}

	printf("\n%d checks, %d failures\n", g_checks, g_failures);
	return g_failures == 0 ? 0 : 1;
}
