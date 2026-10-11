/*
 * Synthetic IMU for the filter tests (TEST_PLAN.md section 3.1).
 *
 * Ground truth is built from Euler angles with formulas written here, never
 * with ekf_math.c, so that a sign error in the filter cannot cancel itself out.
 */
#ifndef SIM_H_
#define SIM_H_

#include <stdint.h>

#define SIM_DT_S       (1.0f / 104.0f)
#define SIM_DEG        0.017453292519943f
#define SIM_FIELD_NORM 0.4717f
#define SIM_FIELD_DIP  (57.99f * SIM_DEG)

/* Fills roll/pitch/yaw [rad] and their time derivatives [rad/s] at time t. */
typedef void (*sim_motion_fn)(float t, float euler[3], float euler_rate[3]);

/* Adds disturbances to the ideal body-frame readings at time t. */
typedef void (*sim_disturbance_fn)(float t, float accel_b[3], float mag_b[3]);

struct sim {
	sim_motion_fn motion;
	sim_disturbance_fn disturbance; /* may be NULL */
	float gyro_limit;               /* clip level in rad/s; 0 disables clipping */
	uint32_t seed;
	uint32_t step;
};

struct sim_sample {
	float t;
	float euler[3]; /* true roll, pitch, yaw */
	float gyro[3];
	float accel[3];
	float mag[3];
};

extern const float sim_true_bias[3];
extern const float sim_field_n[3]; /* gauss, NED */

void sim_init(struct sim *sim, sim_motion_fn motion, sim_disturbance_fn disturbance);
void sim_next(struct sim *sim, struct sim_sample *sample);

float sim_uniform(uint32_t *seed);
float sim_randn(uint32_t *seed);

void sim_rotation_matrix(const float euler[3], float rot[3][3]);
float sim_wrap_deg(float angle_deg);

#endif /* SIM_H_ */
