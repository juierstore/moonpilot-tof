/****************************************************************************
 * TEST ONLY module - not part of the delivered firmware.
 *
 * Writes Nooploop TOFSense-MS (NLink) frames derived from the SIH ground truth
 * altitude into a file. A host side relay copies the bytes into a pty so that
 * the real tof_sense_ms serial driver can be exercised in a closed loop:
 *
 *   SIH ground truth -> this module -> file -> relay -> pty -> tof_sense_ms
 *                    -> distance_sensor -> EKF2 -> controller -> SIH
 ****************************************************************************/

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>

#include <cstdio>
#include <cstring>

using namespace time_literals;

namespace
{
constexpr int kZones = 64;
constexpr int kFrameLength = 400;

uint8_t checksum8(const uint8_t *data, int length)
{
	uint8_t sum = 0;

	for (int i = 0; i < length; i++) {
		sum = static_cast<uint8_t>(sum + data[i]);
	}

	return sum;
}
}

class TofGtFrames : public px4::ScheduledWorkItem
{
public:
	TofGtFrames(const char *path) :
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
	{
		strncpy(_path, path, sizeof(_path) - 1);
		_path[sizeof(_path) - 1] = '\0';
	}

	~TofGtFrames() override
	{
		ScheduleClear();

		if (_file) {
			fclose(_file);
		}
	}

	int init()
	{
		_file = fopen(_path, "wb");

		if (_file == nullptr) {
			PX4_ERR("cannot open %s", _path);
			return PX4_ERROR;
		}

		setvbuf(_file, nullptr, _IONBF, 0);

		ScheduleOnInterval(100_ms);   // 10 Hz, same as the real sensor default

		return PX4_OK;
	}

	void print_info()
	{
		PX4_INFO("path: %s, frames: %" PRIu32 ", last truth z: %.3f m", _path, _frames, (double)_z);
	}

private:
	void Run() override
	{
		if (_sub.updated()) {
			_sub.copy(&_position);
		}

		_z = _position.z;

		float distance = -_z;                                  // NED: z is negative up

		// small deterministic noise so the estimator sees a real sensor
		distance += static_cast<float>((_frame_id % 5) - 2) * 0.005f;

		if (distance < 0.02f) {
			distance = 0.02f;
		}

		if (distance > 4.0f) {
			distance = 4.0f;
		}

		build_frame(distance);

		if (_file) {
			fwrite(_frame, 1, kFrameLength, _file);
			fflush(_file);
			_frames++;
		}

		_frame_id++;
	}

	void build_frame(float distance_m)
	{
		const uint32_t distance_um = static_cast<uint32_t>(distance_m * 1e6f);
		const uint32_t time_ms = static_cast<uint32_t>(hrt_absolute_time() / 1000);

		int i = 0;
		_frame[i++] = 0x57;
		_frame[i++] = 0x01;
		_frame[i++] = 0xFF;
		_frame[i++] = 0x00;                                   // id
		_frame[i++] = time_ms & 0xFF;
		_frame[i++] = (time_ms >> 8) & 0xFF;
		_frame[i++] = (time_ms >> 16) & 0xFF;
		_frame[i++] = (time_ms >> 24) & 0xFF;
		_frame[i++] = kZones;

		for (int z = 0; z < kZones; z++) {
			_frame[i++] = distance_um & 0xFF;
			_frame[i++] = (distance_um >> 8) & 0xFF;
			_frame[i++] = (distance_um >> 16) & 0xFF;
			_frame[i++] = 0x00;                           // status: valid
			_frame[i++] = static_cast<uint8_t>(100 + (z % 40));
			_frame[i++] = 0x00;
		}

		for (int t = 0; t < 6; t++) {
			_frame[i++] = 0xFF;                           // trailer
		}

		_frame[i] = checksum8(_frame, kFrameLength - 1);
	}

	uORB::Subscription _sub{ORB_ID(vehicle_local_position_groundtruth)};
	vehicle_local_position_s _position{};

	char _path[80] {};
	FILE *_file{nullptr};
	uint8_t _frame[kFrameLength] {};
	uint8_t _frame_id{0};
	uint32_t _frames{0};
	float _z{0.f};
};

namespace tof_gt_frames
{
TofGtFrames *g_dev{nullptr};

int start(const char *path);
int stop();
int usage();
int status();

int start(const char *path)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	g_dev = new TofGtFrames(path);

	if (g_dev == nullptr) {
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	delete g_dev;
	g_dev = nullptr;
	return PX4_OK;
}

int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	g_dev->print_info();
	return PX4_OK;
}

int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

TEST ONLY. Writes Nooploop TOFSense-M/MS NLink frames built from the SIH ground
truth altitude into a file, for closed loop testing of the tof_sense_ms driver.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tof_gt_frames", "test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start");
	PRINT_MODULE_USAGE_PARAM_STRING('f', "/tmp/tof_frames.bin", nullptr, "output file", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Status");
	return PX4_OK;
}
} // namespace tof_gt_frames

extern "C" __EXPORT int tof_gt_frames_main(int argc, char *argv[])
{
	if (argc < 2) {
		return tof_gt_frames::usage();
	}

	if (!strcmp(argv[1], "start")) {
		const char *path = "/tmp/tof_frames.bin";
		int myoptind = 2;
		const char *myoptarg = nullptr;
		int ch = 0;

		while ((ch = px4_getopt(argc, argv, "f:", &myoptind, &myoptarg)) != EOF) {
			if (ch == 'f') {
				path = myoptarg;
			}
		}

		return tof_gt_frames::start(path);
	}

	if (!strcmp(argv[1], "stop")) {
		return tof_gt_frames::stop();
	}

	if (!strcmp(argv[1], "status")) {
		return tof_gt_frames::status();
	}

	return tof_gt_frames::usage();
}
