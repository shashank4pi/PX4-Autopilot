############################################################################
#
#   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_romfs_files(

	1010_gazebo-classic_iris_opt_flow
	1010_gazebo-classic_iris_opt_flow.post
	1011_gazebo-classic_iris_irlock
	1012_gazebo-classic_iris_rplidar
	1013_gazebo-classic_iris_vision
	1013_gazebo-classic_iris_vision.post
	1015_gazebo-classic_iris_depth_camera
	1016_gazebo-classic_iris_downward_depth_camera
	1017_gazebo-classic_iris_opt_flow_mockup
	1019_gazebo-classic_iris_dual_gps
	1021_gazebo-classic_uuv_hippocampus
	1022_gazebo-classic_uuv_bluerov2_heavy
	1030_gazebo-classic_plane
	1031_gazebo-classic_plane_cam
	1031_gazebo-classic_plane_cam.post
	1032_gazebo-classic_plane_catapult
	1033_jsbsim_rascal
	1034_flightgear_rascal-electric
	1035_gazebo-classic_techpod
	1036_jsbsim_malolo
	1037_gazebo-classic_believer
	1038_gazebo-classic_glider
	1039_gazebo-classic_advanced_plane
	1040_gazebo-classic_standard_vtol
	1041_gazebo-classic_tailsitter
	1042_gazebo-classic_tiltrotor
	1043_gazebo-classic_standard_vtol_drop
	1044_gazebo-classic_plane_lidar
	1045_gazebo-classic_quadtailsitter
	1062_flightgear_tf-r1
	1070_gazebo-classic_boat

	2507_gazebo-classic_cloudship

	3010_jsbsim_quadrotor_x
	3011_jsbsim_hexarotor_x

	4001_gz_x500
	4002_gz_x500_depth
	4003_gz_rc_cessna
	4004_gz_standard_vtol
	4005_gz_x500_vision
	4006_gz_px4vision
	4008_gz_advanced_plane
	4009_gz_r1_rover
	4010_gz_x500_mono_cam
	4011_gz_lawnmower
	4013_gz_x500_lidar_2d
	4014_gz_x500_mono_cam_down
	4016_gz_x500_lidar_down
	4017_gz_x500_lidar_front
	4018_gz_quadtailsitter
	4019_gz_x500_gimbal
	4020_gz_tiltrotor
	4021_gz_x500_flow

	6011_gazebo-classic_typhoon_h480
	6011_gazebo-classic_typhoon_h480.post

	8011_gz_omnicopter

	10015_gazebo-classic_iris
	10016_none_iris
	10017_jmavsim_iris
	10018_gazebo-classic_iris_foggy_lidar
	10019_gazebo-classic_omnicopter
	10030_gazebo-classic_px4vision

	10040_sihsim_quadx
	10041_sihsim_airplane
	10042_sihsim_xvert
	10043_sihsim_standard_vtol
	10044_sihsim_hex
	10045_sihsim_rover_ackermann

	17001_flightgear_tf-g1
	17002_flightgear_tf-g2

	50000_gz_rover_differential
	51000_gz_rover_ackermann
	52000_gz_rover_mecanum

	60002_gz_uuv_bluerov2_heavy

	70000_gz_atmos
	70001_gz_atmos_dual

	# [22000, 22999] Reserve for custom models
)

# --- astrm overlay hook --------------------------------------------------------
# Auto-discover any *_gz_* airframes dropped into this directory by
# astrm_dronesim's overlay catalog (drones/_lib/sync_overlay.sh symlinks
# drones/overlay/airframes/* here so CMake sees them).  Files already
# enumerated above are skipped — this only adds genuinely-new overlay
# entries to PX4_ROMFS_FILES so they get copied into rootfs at build
# time and PX4 SITL can find them at runtime.  See
# astrm_vGPS/scripts/sim/link_overlay_airframes.sh and
# astrm_dronesim/drones/README.md for the overlay design.
file(GLOB _astrm_overlay_gz RELATIVE ${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_CURRENT_LIST_DIR}/*_gz_*)
get_property(_astrm_already_added GLOBAL PROPERTY PX4_ROMFS_FILES)
foreach(_astrm_af IN LISTS _astrm_overlay_gz)
	set(_astrm_full "${CMAKE_CURRENT_LIST_DIR}/${_astrm_af}")
	list(FIND _astrm_already_added "${_astrm_full}" _astrm_idx)
	if(_astrm_idx EQUAL -1)
		message(STATUS "astrm overlay airframe: ${_astrm_af}")
		px4_add_romfs_files(${_astrm_af})
	endif()
endforeach()
