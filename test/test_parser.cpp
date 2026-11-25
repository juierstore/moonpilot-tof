/**
 * Host side unit test for the TOFSense NLink parser.
 *
 * The parser only depends on <cstdint>, so it can be regression tested on any
 * PC without a PX4 toolchain:
 *
 *   cd test
 *   make test        # or: make && ./test_parser
 *
 * Covered cases:
 *   - the 400 byte reference frame from the Nooploop documentation (8x8)
 *   - single point frames (positive / negative distance / invalid status)
 *   - synthetic 8x8 frames, with and without the 6 byte trailer
 *   - 4x4 frames (16 zones), with and without trailer
 *   - checksum corruption and resynchronisation on the next frame
 *   - truncated frames, garbage bytes, invalid function marks, invalid zone map
 *   - parser reset, byte-by-byte vs chunked feeding, 0x57 inside the payload
 */

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../src/drivers/distance_sensor/tof_sense_ms/tof_sense_ms_parser.h"
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
static std::vector<uint8_t> make_point_frame(int32_t distance_mm, uint8_t status = 0,
		uint16_t signal = 3)
{
	std::vector<uint8_t> f;
	f.push_back(0x57);
	f.push_back(0x00);
	f.push_back(0xFF);
	f.push_back(0x00);              // id
	append_u32le(f, 36766);         // system time
	append_u24le(f, static_cast<uint32_t>(distance_mm) & 0xFFFFFF);
	f.push_back(status);
	append_u16le(f, signal);
	f.push_back(0x06);              // range precision
	f.push_back(checksum8(f.data(), f.size()));
	return f;
}

/* M/MS series frame: 9 + 6*pixels (+6 trailer) + 1 */
static std::vector<uint8_t> make_pixel_frame(int pixels, bool trailer,
		const std::vector<int32_t> &distances_mm,
		const std::vector<uint8_t> *statuses = nullptr,
		const std::vector<uint16_t> *signals = nullptr)
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
		f.push_back(statuses ? (*statuses)[i] : 0x00);
		append_u16le(f, signals ? (*signals)[i] : static_cast<uint16_t>(100 + i));
	}

	if (trailer) {
		for (int i = 0; i < 6; i++) {
			f.push_back(0xFF);
		}
	}

	f.push_back(checksum8(f.data(), f.size()));
	return f;
}

static std::vector<int32_t> constant_distances(int pixels, int32_t distance_mm)
{
	return std::vector<int32_t>(static_cast<size_t>(pixels), distance_mm);
}

static int feed(Parser &parser, Frame &frame, const std::vector<uint8_t> &bytes)
{
	int decoded = 0;

	for (uint8_t b : bytes) {
		if (parser.push(b, frame)) {
			decoded++;
		}
	}

	return decoded;
}

int main()
{
	printf("TOFSense NLink parser test\n");

	/* 1. reference frame from the Nooploop protocol documentation */
	{
		printf("[1] reference 8x8 frame from the vendor documentation\n");
		std::vector<uint8_t> bytes(kSampleFrame, kSampleFrame + kSampleFrameSize);
		CHECK(bytes.size() == 400, "sample frame is %zu bytes, expected 400", bytes.size());
		CHECK(checksum8(bytes.data(), bytes.size() - 1) == bytes.back(), "vendor frame checksum");

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes) == 1, "vendor frame not decoded");
		CHECK(parser.frames() == 1, "frames %u", parser.frames());
		CHECK(parser.errors() == 0, "errors %u", parser.errors());
		CHECK(parser.checksum_errors() == 0, "checksum errors %u", parser.checksum_errors());
		CHECK(frame.type == 1, "type %u", frame.type);
		CHECK(frame.pixel_count == 64, "zones %u", frame.pixel_count);
		CHECK(frame.pixels[0].distance_mm == 492, "zone0 %d mm, expected 492", frame.pixels[0].distance_mm);
		CHECK(frame.pixels[1].distance_mm == 214, "zone1 %d mm, expected 214", frame.pixels[1].distance_mm);
		CHECK(frame.pixels[63].distance_mm == 26, "zone63 %d mm, expected 26", frame.pixels[63].distance_mm);
		CHECK(frame.pixels[0].signal == 159, "zone0 signal %u, expected 159", frame.pixels[0].signal);
		CHECK(frame.pixels[0].status == 0, "zone0 status %u", frame.pixels[0].status);

		int invalid_zones = 0;

		for (int i = 0; i < frame.pixel_count; i++) {
			if (frame.pixels[i].status != 0) {
				invalid_zones++;
			}
		}

		CHECK(invalid_zones > 0, "vendor frame should contain invalid zones");
	}

	/* 2. single point series */
	{
		printf("[2] single point frame\n");
		Parser parser;
		Frame frame;
		const auto bytes = make_point_frame(2221, 0, 3);
		CHECK(bytes.size() == 16, "point frame size %zu", bytes.size());
		CHECK(feed(parser, frame, bytes) == 1, "point frame not decoded");
		CHECK(frame.type == 0, "type %u", frame.type);
		CHECK(frame.pixel_count == 1, "zones %u", frame.pixel_count);
		CHECK(frame.pixels[0].distance_mm == 2221, "distance %d mm", frame.pixels[0].distance_mm);
		CHECK(frame.pixels[0].signal == 3, "signal %u", frame.pixels[0].signal);
		CHECK(frame.pixels[0].status == 0, "status %u", frame.pixels[0].status);
		CHECK(parser.errors() == 0, "errors %u", parser.errors());
	}

	/* 3. negative distance of the point series */
	{
		printf("[3] single point negative distance\n");
		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, make_point_frame(-1044, 0, 3)) == 1, "point frame not decoded");
		CHECK(frame.pixels[0].distance_mm == -1044, "distance %d mm", frame.pixels[0].distance_mm);
	}

	/* 4. point frame with a sensor status flag (invalid measurement) */
	{
		printf("[4] single point frame with status != 0\n");
		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, make_point_frame(1000, 0xFF, 3)) == 1, "point frame not decoded");
		CHECK(frame.pixels[0].status == 0xFF, "status %u", frame.pixels[0].status);
		CHECK(frame.pixels[0].distance_mm == 1000, "distance %d mm", frame.pixels[0].distance_mm);
	}

	/* 5. synthetic 8x8 frame */
	{
		printf("[5] synthetic 8x8 frame\n");
		std::vector<int32_t> distances;

		for (int i = 0; i < 64; i++) {
			distances.push_back(2000 + i);
		}

		const auto bytes = make_pixel_frame(64, true, distances);
		CHECK(bytes.size() == 400, "frame size %zu", bytes.size());

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes) == 1, "8x8 frame not decoded");
		CHECK(frame.pixel_count == 64, "zones %u", frame.pixel_count);
		CHECK(frame.pixels[0].distance_mm == 2000, "zone0 %d", frame.pixels[0].distance_mm);
		CHECK(frame.pixels[63].distance_mm == 2063, "zone63 %d", frame.pixels[63].distance_mm);
	}

	/* 6. 4x4 frame without the 6 byte trailer */
	{
		printf("[6] 4x4 frame, no trailer\n");
		const auto bytes = make_pixel_frame(16, false, constant_distances(16, 1500));
		CHECK(bytes.size() == 106, "frame size %zu", bytes.size());

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes) == 1, "4x4 frame (no trailer) not decoded");
		CHECK(frame.pixel_count == 16, "zones %u", frame.pixel_count);
		CHECK(frame.pixels[15].distance_mm == 1500, "zone15 %d", frame.pixels[15].distance_mm);
	}

	/* 7. 4x4 frame with the 6 byte trailer */
	{
		printf("[7] 4x4 frame with trailer\n");
		const auto bytes = make_pixel_frame(16, true, constant_distances(16, 1500));
		CHECK(bytes.size() == 112, "frame size %zu", bytes.size());

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes) == 1, "4x4 frame (trailer) not decoded");
		CHECK(frame.pixel_count == 16, "zones %u", frame.pixel_count);
	}

	/* 8. corrupted frame must be rejected and the parser must resync */
	{
		printf("[8] corrupted frame + resync\n");
		Parser parser;
		Frame frame;
		auto bytes = make_pixel_frame(64, true, constant_distances(64, 1234));
		bytes[100] ^= 0xFF;             // break the payload

		CHECK(feed(parser, frame, bytes) == 0, "corrupted frame was accepted");
		CHECK(parser.checksum_errors() == 1, "checksum errors %u", parser.checksum_errors());
		CHECK(parser.header_errors() == 0, "header errors %u", parser.header_errors());

		// A valid frame following the garbage must still be decoded.
		CHECK(feed(parser, frame, make_pixel_frame(64, true, constant_distances(64, 1234))) == 1,
		      "no resync after a corrupted frame");
		CHECK(parser.frames() == 1, "frames %u after resync", parser.frames());
	}

	/* 9. truncated frame followed by a complete frame */
	{
		printf("[9] truncated frame + resync\n");
		Parser parser;
		Frame frame;
		auto bytes = make_pixel_frame(64, true, constant_distances(64, 1000));
		std::vector<uint8_t> partial(bytes.begin(), bytes.begin() + 200);
		CHECK(feed(parser, frame, partial) == 0, "partial frame was accepted");
		CHECK(parser.frames() == 0, "frames %u", parser.frames());

		// The parser has to drop the stale 200 byte fragment and pick up a
		// following frame; the first good frame completes/discards the
		// fragment, the second one is decoded.
		std::vector<uint8_t> two_frames = bytes;
		two_frames.insert(two_frames.end(), bytes.begin(), bytes.end());
		CHECK(feed(parser, frame, two_frames) >= 1, "no resync after a truncated frame");
		CHECK(frame.pixels[0].distance_mm == 1000, "zone0 %d", frame.pixels[0].distance_mm);
	}

	/* 10. bad function mark after the header */
	{
		printf("[10] invalid function mark + resync\n");
		Parser parser;
		Frame frame;
		const std::vector<uint8_t> bad = {0x57, 0x42, 0xFF, 0x00};
		CHECK(feed(parser, frame, bad) == 0, "bad function mark was accepted");
		CHECK(parser.header_errors() == 1, "header errors %u", parser.header_errors());

		CHECK(feed(parser, frame, make_point_frame(1234)) == 1, "no resync after a bad function mark");
		CHECK(parser.frames() == 1, "frames %u", parser.frames());
	}

	/* 11. invalid zone map (zero zones) */
	{
		printf("[11] invalid zone map + resync\n");
		Parser parser;
		Frame frame;
		std::vector<uint8_t> bad = {0x57, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		CHECK(feed(parser, frame, bad) == 0, "zero zone frame was accepted");
		CHECK(parser.header_errors() == 1, "header errors %u", parser.header_errors());

		CHECK(feed(parser, frame, make_pixel_frame(64, false, constant_distances(64, 500))) == 1,
		      "no resync after an invalid zone map");
		CHECK(parser.frames() == 1, "frames %u", parser.frames());
	}

	/* 12. garbage before the header is ignored without counting errors */
	{
		printf("[12] garbage before the header\n");
		Parser parser;
		Frame frame;
		std::vector<uint8_t> bytes = {0x00, 0x12, 0x34, 0xFF, 0x56};
		CHECK(feed(parser, frame, bytes) == 0, "garbage decoded as a frame");

		const auto good = make_point_frame(1234);
		bytes.insert(bytes.end(), good.begin(), good.end());
		CHECK(feed(parser, frame, bytes) == 1, "frame after garbage not decoded");
		CHECK(parser.errors() == 0, "garbage counted as an error (%u)", parser.errors());
	}

	/* 13. reset() drops a partially received frame */
	{
		printf("[13] parser reset\n");
		Parser parser;
		Frame frame;
		auto bytes = make_pixel_frame(64, true, constant_distances(64, 800));
		std::vector<uint8_t> half(bytes.begin(), bytes.begin() + 200);
		CHECK(feed(parser, frame, half) == 0, "partial frame decoded");
		parser.reset();
		CHECK(feed(parser, frame, bytes) == 1, "frame after reset not decoded");
		CHECK(frame.pixels[0].distance_mm == 800, "zone0 %d", frame.pixels[0].distance_mm);
	}

	/* 14. one byte at a time vs. chunked feeding */
	{
		printf("[14] byte-by-byte vs chunked feeding\n");
		Parser parser_a;
		Parser parser_b;
		Frame frame_a;
		Frame frame_b;
		std::vector<uint8_t> stream;

		for (int i = 0; i < 5; i++) {
			const auto point = make_point_frame(1000 + i);
			stream.insert(stream.end(), point.begin(), point.end());
			const auto pixel = make_pixel_frame(64, true, constant_distances(64, 2000 + i));
			stream.insert(stream.end(), pixel.begin(), pixel.end());
		}

		int frames_a = feed(parser_a, frame_a, stream);
		int frames_b = 0;

		// Feed in irregular chunks, including the same parser state transitions.
		size_t offset = 0;
		const size_t chunk_sizes[] = {1, 3, 7, 1, 11, 2, 13};

		while (offset < stream.size()) {
			const size_t chunk = chunk_sizes[(offset / 3) % (sizeof(chunk_sizes) / sizeof(chunk_sizes[0]))];
			std::vector<uint8_t> part(stream.begin() + offset,
						  stream.begin() + std::min(stream.size(), offset + chunk));
			frames_b += feed(parser_b, frame_b, part);
			offset += part.size();
		}

		CHECK(frames_a == 10, "byte-wise frames %d", frames_a);
		CHECK(frames_b == 10, "chunked frames %d", frames_b);
		CHECK(parser_a.frames() == parser_b.frames(), "frame counters differ");
		CHECK(parser_a.errors() == 0 && parser_b.errors() == 0, "errors %u/%u",
		      parser_a.errors(), parser_b.errors());
	}

	/* 15. oversized garbage must not overflow the parser buffer */
	{
		printf("[15] oversized garbage\n");
		Parser parser;
		Frame frame;
		std::vector<uint8_t> garbage;
		garbage.push_back(0x57);
		garbage.push_back(0x01);
		garbage.push_back(0xFF);

		for (int i = 0; i < 600; i++) {
			garbage.push_back(static_cast<uint8_t>(i * 7));
		}

		CHECK(feed(parser, frame, garbage) == 0, "oversized garbage decoded as a frame");
		CHECK(parser.errors() > 0, "oversized garbage not counted");

		CHECK(feed(parser, frame, make_point_frame(4321)) == 1, "no resync after oversized garbage");
		CHECK(frame.pixels[0].distance_mm == 4321, "distance %d", frame.pixels[0].distance_mm);
	}

	/* 16. 0x57 bytes inside the payload must not start a new frame */
	{
		printf("[16] 0x57 inside the payload\n");
		std::vector<int32_t> distances = constant_distances(64, 0x57);
		std::vector<uint16_t> signals(64, 0x5757);
		const auto bytes = make_pixel_frame(64, true, distances, nullptr, &signals);

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes) == 1, "frame with 0x57 payload not decoded");
		CHECK(frame.pixels[0].distance_mm == 0x57, "zone0 %d", frame.pixels[0].distance_mm);
		CHECK(frame.pixels[0].signal == 0x5757, "zone0 signal %u", frame.pixels[0].signal);
		CHECK(parser.checksum_errors() == 0, "checksum errors %u", parser.checksum_errors());
	}

	/* 17. all zones marked invalid still yields a valid frame (driver filters) */
	{
		printf("[17] all zones marked invalid\n");
		std::vector<int32_t> distances(64, 0);
		std::vector<uint8_t> statuses(64, 0xFF);
		const auto bytes = make_pixel_frame(64, true, distances, &statuses);

		Parser parser;
		Frame frame;
		CHECK(feed(parser, frame, bytes) == 1, "all-invalid frame not decoded");
		CHECK(frame.pixels[0].status == 0xFF, "status %u", frame.pixels[0].status);
		CHECK(frame.pixels[63].status == 0xFF, "status %u", frame.pixels[63].status);
	}

	printf("\n%d checks, %d failures\n", g_checks, g_failures);

	return g_failures == 0 ? 0 : 1;
}
