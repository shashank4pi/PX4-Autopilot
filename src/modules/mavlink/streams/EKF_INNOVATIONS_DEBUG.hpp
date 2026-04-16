/****************************************************************************
 *
 *   Copyright (c) 2026 astrm_dronesim. Part of the shashank4pi/PX4-Autopilot
 *   fork on branch `astrm_dronesim`. Not upstream.
 *
 *   Redistribution and use subject to the upstream PX4 BSD-3-Clause license.
 *
 ****************************************************************************/

// ---------------------------------------------------------------------------
// Why this stream exists
// ---------------------------------------------------------------------------
// Stock PX4 ships only the aggregated test-ratio scalars (pos_horiz_ratio,
// pos_vert_ratio, vel_ratio, mag_ratio, hagl_ratio) inside ESTIMATOR_STATUS.
// For GNSS-denied / spoof-detection work we want:
//   1. The *signed* per-axis GPS innovations (N, E, D position + velocity) so
//      we can see which direction the attack is pulling the filter, not just
//      that something is inconsistent.
//   2. The per-axis innovation test ratios, not just a single rolled-up value
//      per sensor family.
//   3. Mag / baro / flow / heading innovations in the same shape so every
//      aiding source is observable from one bridge handler.
//
// Rather than inventing a new MAVLink message ID (which would diverge from
// the spec), we piggyback on DEBUG_FLOAT_ARRAY — it has 58 float slots and a
// 10-char name, which is plenty. On successive send() calls we alternate the
// payload between the innovations topic and the test-ratios topic, tagging
// them with distinct `name` fields ("ekf_innov" / "ekf_ratio") so the ground
// side can route by name.
//
// Wire identity: this stream registers as "EKF_INNOVATIONS_DEBUG" (a PX4
// stream-name, used by `mavlink stream` / SET_MESSAGE_INTERVAL) but sends
// MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY over the wire, the same ID the stock
// DEBUG_FLOAT_ARRAY stream uses. Two PX4 streams can coexist at the same
// msg-id — the scheduler keys them by name. SET_MESSAGE_INTERVAL targets
// msg-id and therefore affects both, which is fine for us.
// ---------------------------------------------------------------------------

#ifndef EKF_INNOVATIONS_DEBUG_HPP
#define EKF_INNOVATIONS_DEBUG_HPP

#include <uORB/topics/estimator_innovations.h>
#include <cstring>
#include <cmath>

class MavlinkStreamEkfInnovationsDebug : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamEkfInnovationsDebug(mavlink); }

	// Stream-name visible to `mavlink stream -s EKF_INNOVATIONS_DEBUG`.
	// Deliberately distinct from the stock DEBUG_FLOAT_ARRAY so both can
	// be enabled independently.
	static constexpr const char *get_name_static() { return "EKF_INNOVATIONS_DEBUG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	// Worst-case size estimate: we emit up to 2 messages per cycle.
	unsigned get_size() override
	{
		return (_innov_sub.advertised() || _ratio_sub.advertised())
			? 2 * (MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES)
			: 0;
	}

private:
	explicit MavlinkStreamEkfInnovationsDebug(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _innov_sub{ORB_ID(estimator_innovations)};
	uORB::Subscription _ratio_sub{ORB_ID(estimator_innovation_test_ratios)};

	// ---- shared layout: index into debug_float_array.data[58] ----
	// Bridge-side code (tools/rerun/live_mavlink/src/main.cpp ::
	// on_debug_float_array) must match this exactly. When extending, append
	// new slots and bump PACKED_LEN rather than reordering — that way old
	// bridges keep working against new firmware.
	enum : uint8_t {
		IX_GPS_HVEL_N = 0, IX_GPS_HVEL_E,
		IX_GPS_VVEL,
		IX_GPS_HPOS_N,  IX_GPS_HPOS_E,
		IX_GPS_VPOS,
		IX_MAG_X,       IX_MAG_Y,       IX_MAG_Z,
		IX_HEADING,
		IX_BARO_VPOS,
		IX_HAGL,
		IX_HAGL_RATE,
		IX_FLOW_X,      IX_FLOW_Y,
		IX_AIRSPEED,
		IX_BETA,
		IX_EV_HVEL_N,   IX_EV_HVEL_E,   IX_EV_VVEL,
		IX_EV_HPOS_N,   IX_EV_HPOS_E,   IX_EV_VPOS,
		PACKED_LEN  // == 23; remaining slots in data[58] stay at NaN
	};

	static void pack(const estimator_innovations_s &s, float out[58])
	{
		for (int i = 0; i < 58; ++i) { out[i] = NAN; }

		out[IX_GPS_HVEL_N] = s.gps_hvel[0];
		out[IX_GPS_HVEL_E] = s.gps_hvel[1];
		out[IX_GPS_VVEL]   = s.gps_vvel;
		out[IX_GPS_HPOS_N] = s.gps_hpos[0];
		out[IX_GPS_HPOS_E] = s.gps_hpos[1];
		out[IX_GPS_VPOS]   = s.gps_vpos;

		out[IX_MAG_X] = s.mag_field[0];
		out[IX_MAG_Y] = s.mag_field[1];
		out[IX_MAG_Z] = s.mag_field[2];
		out[IX_HEADING] = s.heading;

		out[IX_BARO_VPOS] = s.baro_vpos;
		out[IX_HAGL]      = s.hagl;
		out[IX_HAGL_RATE] = s.hagl_rate;

		out[IX_FLOW_X] = s.flow[0];
		out[IX_FLOW_Y] = s.flow[1];

		out[IX_AIRSPEED] = s.airspeed;
		out[IX_BETA]     = s.beta;

		out[IX_EV_HVEL_N] = s.ev_hvel[0];
		out[IX_EV_HVEL_E] = s.ev_hvel[1];
		out[IX_EV_VVEL]   = s.ev_vvel;
		out[IX_EV_HPOS_N] = s.ev_hpos[0];
		out[IX_EV_HPOS_E] = s.ev_hpos[1];
		out[IX_EV_VPOS]   = s.ev_vpos;
	}

	// Tagged with distinct array_id + name so the ground station can route
	// without guessing. Keep id values stable — bridges pin on them too.
	static constexpr uint16_t ARRAY_ID_INNOV = 10;
	static constexpr uint16_t ARRAY_ID_RATIO = 11;

	void send_one(uint16_t array_id, const char *name, const float data[58], uint64_t ts)
	{
		mavlink_debug_float_array_t msg{};
		msg.time_usec = ts;
		msg.array_id = array_id;
		std::strncpy(msg.name, name, sizeof(msg.name));
		msg.name[sizeof(msg.name) - 1] = '\0';
		for (size_t i = 0; i < 58; ++i) { msg.data[i] = data[i]; }
		mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);
	}

	bool send() override
	{
		bool sent = false;

		estimator_innovations_s innov;
		if (_innov_sub.update(&innov)) {
			float buf[58];
			pack(innov, buf);
			send_one(ARRAY_ID_INNOV, "ekf_innov", buf, innov.timestamp);
			sent = true;
		}

		estimator_innovations_s ratios;
		if (_ratio_sub.update(&ratios)) {
			float buf[58];
			pack(ratios, buf);
			send_one(ARRAY_ID_RATIO, "ekf_ratio", buf, ratios.timestamp);
			sent = true;
		}

		return sent;
	}
};

#endif // EKF_INNOVATIONS_DEBUG_HPP
