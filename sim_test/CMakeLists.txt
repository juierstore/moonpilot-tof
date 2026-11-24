############################################################################
# TEST ONLY module - closed loop SITL helper for the TOF driver
############################################################################
px4_add_module(
	MODULE modules__tof_gt_frames
	MAIN tof_gt_frames
	SRCS
		tof_gt_frames.cpp
	)
