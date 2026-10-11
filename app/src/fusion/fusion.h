/*
 * Fusion thread: alignment state machine around the EKF and the snapshot that
 * telemetry and the shell read. The EKF itself belongs to this thread alone;
 * other threads only leave requests that are applied at the next sample.
 */
#ifndef FUSION_H_
#define FUSION_H_

#include <stdbool.h>
#include <stdint.h>

#include "ekf/ekf_core.h"

enum fusion_state {
	FUSION_ALIGNING = 0, /* collecting the alignment window */
	FUSION_RUNNING,
};

#define FUSION_FLAG_HEADING_CONVERGED (1U << 0)
#define FUSION_FLAG_YAW_VALID         (1U << 1)
#define FUSION_FLAG_MAG_ACCEPTED      (1U << 2) /* this sample's heading update was applied */

struct fusion_stats {
	uint32_t ekf_input_faults;   /* samples the EKF refused with -EINVAL */
	uint32_t ekf_numeric_faults; /* -EDOM from the EKF; each one forces a re-alignment */
	uint32_t align_restarts;     /* returns from RUNNING to ALIGNING, faults and requests */
};

struct fusion_snapshot {
	uint32_t seq;          /* samples processed so far */
	uint32_t timestamp_ms; /* uptime at publication */
	enum fusion_state state;
	float q[4];            /* body to NED, as the filter holds it (sign not canonical) */
	float gyro_bias[3];    /* rad/s, body frame */
	struct ekf_stats ekf;
	struct fusion_stats fusion;
	uint32_t ekf_cyc;      /* cycles spent in predict and updates for this sample */
	uint32_t latency_cyc;  /* cycles from the DRDY timestamp to the end of the EKF step */
	uint32_t flags;        /* FUSION_FLAG_* */
};

/** Fusion thread entry; needs K_FP_REGS. */
void fusion_run(void *p1, void *p2, void *p3);

/** Copy the latest consistent snapshot; callable from any thread. */
void fusion_get_snapshot(struct fusion_snapshot *out);

/** Ask the fusion thread to drop the current attitude and align again. */
void fusion_request_align(void);

/**
 * Switch the heading correction at run time. It stays off regardless while no
 * magnetometer calibration exists or CONFIG_APP_EKF_USE_MAG is disabled.
 */
void fusion_set_use_mag(bool enable);

#endif /* FUSION_H_ */
