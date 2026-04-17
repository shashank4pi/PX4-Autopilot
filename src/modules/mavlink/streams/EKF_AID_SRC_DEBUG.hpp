/****************************************************************************
 *
 *   Copyright (c) 2026 astrm_dronesim. Part of the shashank4pi/PX4-Autopilot
 *   fork on branch `astrm_dronesim`. Not upstream.
 *
 *   Redistribution and use subject to the upstream PX4 BSD-3-Clause license.
 *
 ****************************************************************************/

// ---------------------------------------------------------------------------
// Why this stream exists (sibling of EKF_INNOVATIONS_DEBUG)
// ---------------------------------------------------------------------------
// The legacy `estimator_innovations` topic (consumed by EKF_INNOVATIONS_DEBUG)
// is frozen around an old, pre-aid-source EKF layout. Modern ekf2 publishes
// per-aid-source state via `estimator_aid_src_*` topics (GNSS pos/vel/hgt, EV
// pos/vel/hgt/yaw, mag, baro, rng, AGP multi-instance, ...). For spoof /
// denial research we want *each* aiding source surfaced to the ground-side
// Rerun bridge with its full state (observation, innovation, variances, test
// ratios, fused/rejected flags), not just a rolled-up legacy view.
//
// Wire shape: MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, one message per update per
// aid source. array_id = 20 for every message emitted by this stream; ground
// side routes by the 10-char `name` tag (e.g. "gnss_pos", "agp", "mag").
//
// Stream-name is "EKF_AID_SRC_DEBUG" — distinct from EKF_INNOVATIONS_DEBUG so
// `mavlink stream` / configure_stream_local can toggle them independently.
// Both streams and the stock DEBUG_FLOAT_ARRAY stream share the same msg-id,
// which means SET_MESSAGE_INTERVAL from a GCS cannot reach the siblings —
// only a name-based configure_stream_local() default in mavlink_main.cpp
// actually turns them on. See the dual-stream gotcha note in that file.
// ---------------------------------------------------------------------------

#ifndef EKF_AID_SRC_DEBUG_HPP
#define EKF_AID_SRC_DEBUG_HPP

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>

#include <cstring>
#include <cmath>

class MavlinkStreamEkfAidSrcDebug : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamEkfAidSrcDebug(mavlink); }

	static constexpr const char *get_name_static() { return "EKF_AID_SRC_DEBUG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	// Worst-case: 4 AGP slots + 10 single-instance aid sources = 14 messages
	// per send() cycle. Scheduler uses this for bandwidth accounting.
	unsigned get_size() override
	{
		return 14 * (MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamEkfAidSrcDebug(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	// ---- shared data[58] layout (ground bridge must match exactly) ----
	// [0]    dim (1/2/3)
	// [1]    instance (uORB multi-instance index; 0 for single-instance topics)
	// [2]    fused (0/1)
	// [3]    innovation_rejected (0/1)
	// [4]    estimator_instance
	// [5]    timestamp_sample in seconds
	// [6..8]   observation[3]          (NaN pad unused)
	// [9..11]  observation_variance[3]
	// [12..14] innovation[3]
	// [15..17] innovation_variance[3]
	// [18..20] test_ratio[3]
	// [21..23] test_ratio_filtered[3]
	// [24..26] innovation_filtered[3]
	// [27..57] reserved, NaN
	enum : uint8_t {
		IX_DIM               = 0,
		IX_INSTANCE          = 1,
		IX_FUSED             = 2,
		IX_INNOV_REJ         = 3,
		IX_EST_INST          = 4,
		IX_TS_S              = 5,
		IX_OBS_0             = 6,  // 6..8
		IX_OBS_VAR_0         = 9,  // 9..11
		IX_INNOV_0           = 12, // 12..14
		IX_INNOV_VAR_0       = 15, // 15..17
		IX_TR_0              = 18, // 18..20
		IX_TR_FILT_0         = 21, // 21..23
		IX_INNOV_FILT_0      = 24, // 24..26
	};

	static constexpr uint16_t ARRAY_ID = 20;

	static void init_nan(float out[58])
	{
		for (int i = 0; i < 58; ++i) { out[i] = NAN; }
	}

	void send_one(const char *name, const float data[58], uint64_t ts)
	{
		mavlink_debug_float_array_t msg{};
		msg.time_usec = ts;
		msg.array_id = ARRAY_ID;
		std::strncpy(msg.name, name, sizeof(msg.name));
		msg.name[sizeof(msg.name) - 1] = '\0';
		for (size_t i = 0; i < 58; ++i) { msg.data[i] = data[i]; }
		mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);
	}

	static void pack_common(float out[58], uint8_t dim, uint8_t instance,
				bool fused, bool innovation_rejected,
				uint8_t estimator_instance, uint64_t timestamp_sample)
	{
		out[IX_DIM]         = static_cast<float>(dim);
		out[IX_INSTANCE]    = static_cast<float>(instance);
		out[IX_FUSED]       = fused ? 1.0f : 0.0f;
		out[IX_INNOV_REJ]   = innovation_rejected ? 1.0f : 0.0f;
		out[IX_EST_INST]    = static_cast<float>(estimator_instance);
		out[IX_TS_S]        = static_cast<float>(timestamp_sample) * 1e-6f;
	}

	static void pack_1d(const estimator_aid_source1d_s &s, uint8_t instance, float out[58])
	{
		init_nan(out);
		pack_common(out, 1, instance, s.fused, s.innovation_rejected,
			    s.estimator_instance, s.timestamp_sample);
		out[IX_OBS_0]            = s.observation;
		out[IX_OBS_VAR_0]        = s.observation_variance;
		out[IX_INNOV_0]          = s.innovation;
		out[IX_INNOV_VAR_0]      = s.innovation_variance;
		out[IX_TR_0]             = s.test_ratio;
		out[IX_TR_FILT_0]        = s.test_ratio_filtered;
		out[IX_INNOV_FILT_0]     = s.innovation_filtered;
	}

	static void pack_2d(const estimator_aid_source2d_s &s, uint8_t instance, float out[58])
	{
		init_nan(out);
		pack_common(out, 2, instance, s.fused, s.innovation_rejected,
			    s.estimator_instance, s.timestamp_sample);
		// observation is float64[2] — cast explicitly.
		out[IX_OBS_0 + 0]        = static_cast<float>(s.observation[0]);
		out[IX_OBS_0 + 1]        = static_cast<float>(s.observation[1]);
		out[IX_OBS_VAR_0 + 0]    = s.observation_variance[0];
		out[IX_OBS_VAR_0 + 1]    = s.observation_variance[1];
		out[IX_INNOV_0 + 0]      = s.innovation[0];
		out[IX_INNOV_0 + 1]      = s.innovation[1];
		out[IX_INNOV_VAR_0 + 0]  = s.innovation_variance[0];
		out[IX_INNOV_VAR_0 + 1]  = s.innovation_variance[1];
		out[IX_TR_0 + 0]         = s.test_ratio[0];
		out[IX_TR_0 + 1]         = s.test_ratio[1];
		out[IX_TR_FILT_0 + 0]    = s.test_ratio_filtered[0];
		out[IX_TR_FILT_0 + 1]    = s.test_ratio_filtered[1];
		out[IX_INNOV_FILT_0 + 0] = s.innovation_filtered[0];
		out[IX_INNOV_FILT_0 + 1] = s.innovation_filtered[1];
	}

	static void pack_3d(const estimator_aid_source3d_s &s, uint8_t instance, float out[58])
	{
		init_nan(out);
		pack_common(out, 3, instance, s.fused, s.innovation_rejected,
			    s.estimator_instance, s.timestamp_sample);
		for (int i = 0; i < 3; ++i) {
			out[IX_OBS_0 + i]        = s.observation[i];
			out[IX_OBS_VAR_0 + i]    = s.observation_variance[i];
			out[IX_INNOV_0 + i]      = s.innovation[i];
			out[IX_INNOV_VAR_0 + i]  = s.innovation_variance[i];
			out[IX_TR_0 + i]         = s.test_ratio[i];
			out[IX_TR_FILT_0 + i]    = s.test_ratio_filtered[i];
			out[IX_INNOV_FILT_0 + i] = s.innovation_filtered[i];
		}
	}

	// Single-instance aid-source subscriptions.
	uORB::Subscription _gnss_pos_sub{ORB_ID(estimator_aid_src_gnss_pos)};
	uORB::Subscription _gnss_vel_sub{ORB_ID(estimator_aid_src_gnss_vel)};
	uORB::Subscription _gnss_hgt_sub{ORB_ID(estimator_aid_src_gnss_hgt)};
	uORB::Subscription _ev_pos_sub{ORB_ID(estimator_aid_src_ev_pos)};
	uORB::Subscription _ev_vel_sub{ORB_ID(estimator_aid_src_ev_vel)};
	uORB::Subscription _ev_hgt_sub{ORB_ID(estimator_aid_src_ev_hgt)};
	uORB::Subscription _ev_yaw_sub{ORB_ID(estimator_aid_src_ev_yaw)};
	uORB::Subscription _mag_sub{ORB_ID(estimator_aid_src_mag)};
	uORB::Subscription _baro_hgt_sub{ORB_ID(estimator_aid_src_baro_hgt)};
	uORB::Subscription _rng_hgt_sub{ORB_ID(estimator_aid_src_rng_hgt)};

	// AGP is published via PublicationMulti (one topic instance per slot,
	// MAX_AGP_IDS == 4). Iterate over all 4 and tag each with its index.
	uORB::SubscriptionMultiArray<estimator_aid_source2d_s, 4> _agp_subs{ORB_ID::estimator_aid_src_aux_global_position};

	bool try_send_1d(uORB::Subscription &sub, const char *name)
	{
		estimator_aid_source1d_s s;
		if (!sub.advertised() || !sub.update(&s)) { return false; }
		float buf[58];
		pack_1d(s, 0, buf);
		send_one(name, buf, s.timestamp);
		return true;
	}

	bool try_send_2d(uORB::Subscription &sub, const char *name)
	{
		estimator_aid_source2d_s s;
		if (!sub.advertised() || !sub.update(&s)) { return false; }
		float buf[58];
		pack_2d(s, 0, buf);
		send_one(name, buf, s.timestamp);
		return true;
	}

	bool try_send_3d(uORB::Subscription &sub, const char *name)
	{
		estimator_aid_source3d_s s;
		if (!sub.advertised() || !sub.update(&s)) { return false; }
		float buf[58];
		pack_3d(s, 0, buf);
		send_one(name, buf, s.timestamp);
		return true;
	}

	bool send() override
	{
		bool sent = false;

		// GNSS
		sent |= try_send_2d(_gnss_pos_sub, "gnss_pos");
		sent |= try_send_3d(_gnss_vel_sub, "gnss_vel");
		sent |= try_send_1d(_gnss_hgt_sub, "gnss_hgt");

		// External vision
		sent |= try_send_2d(_ev_pos_sub, "ev_pos");
		sent |= try_send_3d(_ev_vel_sub, "ev_vel");
		sent |= try_send_1d(_ev_hgt_sub, "ev_hgt");
		sent |= try_send_1d(_ev_yaw_sub, "ev_yaw");

		// Mag / baro / rng
		sent |= try_send_3d(_mag_sub, "mag");
		sent |= try_send_1d(_baro_hgt_sub, "baro");
		sent |= try_send_1d(_rng_hgt_sub, "rng_hgt");

		// AGP — multi-instance, one message per slot that updated.
		for (int i = 0; i < _agp_subs.size(); ++i) {
			estimator_aid_source2d_s s;
			if (_agp_subs[i].advertised() && _agp_subs[i].update(&s)) {
				float buf[58];
				pack_2d(s, static_cast<uint8_t>(i), buf);
				send_one("agp", buf, s.timestamp);
				sent = true;
			}
		}

		return sent;
	}
};

#endif // EKF_AID_SRC_DEBUG_HPP
