#include "sim.h"

#include <math.h>
#include <stddef.h>

#define SIM_GYRO_NOISE  6.3e-4f
#define SIM_ACCEL_NOISE 4.2e-3f
#define SIM_MAG_NOISE   2.0e-3f
#define SIM_G0          9.80665f

const float sim_true_bias[3] = { 0.01f, -0.02f, 0.015f };

const float sim_field_n[3] = { 0.25f, 0.0f, 0.40f };

static uint32_t xorshift32(uint32_t *s)
{
	*s ^= *s << 13;
	*s ^= *s >> 17;
	*s ^= *s << 5;
	return *s;
}

float sim_uniform(uint32_t *seed)
{
	return (float)(xorshift32(seed) >> 8) * (1.0f / 16777216.0f);
}

float sim_randn(uint32_t *seed)
{
	float acc = 0.0f;

	for (int i = 0; i < 12; i++) {
		acc += sim_uniform(seed);
	}

	return acc - 6.0f;
}

void sim_rotation_matrix(const float euler[3], float rot[3][3])
{
	float sr = sinf(euler[0]), cr = cosf(euler[0]);
	float sp = sinf(euler[1]), cp = cosf(euler[1]);
	float sy = sinf(euler[2]), cy = cosf(euler[2]);

	/* R = Rz(yaw) Ry(pitch) Rx(roll), body to navigation. */
	rot[0][0] = cy * cp;
	rot[0][1] = cy * sp * sr - sy * cr;
	rot[0][2] = cy * sp * cr + sy * sr;
	rot[1][0] = sy * cp;
	rot[1][1] = sy * sp * sr + cy * cr;
	rot[1][2] = sy * sp * cr - cy * sr;
	rot[2][0] = -sp;
	rot[2][1] = cp * sr;
	rot[2][2] = cp * cr;
}

float sim_wrap_deg(float angle_deg)
{
	while (angle_deg > 180.0f) {
		angle_deg -= 360.0f;
	}
	while (angle_deg <= -180.0f) {
		angle_deg += 360.0f;
	}

	return angle_deg;
}

void sim_init(struct sim *sim, sim_motion_fn motion, sim_disturbance_fn disturbance)
{
	sim->motion = motion;
	sim->disturbance = disturbance;
	sim->gyro_limit = 0.0f;
	sim->seed = 0x12345678U;
	sim->step = 0;
}

static void body_rate(const float euler[3], const float euler_rate[3], float rate_b[3])
{
	float sr = sinf(euler[0]), cr = cosf(euler[0]);
	float sp = sinf(euler[1]), cp = cosf(euler[1]);

	rate_b[0] = euler_rate[0] - euler_rate[2] * sp;
	rate_b[1] = euler_rate[1] * cr + euler_rate[2] * sr * cp;
	rate_b[2] = -euler_rate[1] * sr + euler_rate[2] * cr * cp;
}

void sim_next(struct sim *sim, struct sim_sample *sample)
{
	float rot[3][3], mid_euler[3], mid_rate[3], unused[3], rate_b[3];

	sim->step++;
	sample->t = (float)sim->step * SIM_DT_S;

	/* The gyro integrates over the past interval; sample the rate at its midpoint. */
	sim->motion(sample->t - 0.5f * SIM_DT_S, mid_euler, mid_rate);
	body_rate(mid_euler, mid_rate, rate_b);

	sim->motion(sample->t, sample->euler, unused);
	sim_rotation_matrix(sample->euler, rot);

	for (int i = 0; i < 3; i++) {
		sample->accel[i] = -SIM_G0 * rot[2][i];
		sample->mag[i] = rot[0][i] * sim_field_n[0] + rot[1][i] * sim_field_n[1] +
				 rot[2][i] * sim_field_n[2];
	}
	if (sim->disturbance != NULL) {
		sim->disturbance(sample->t, sample->accel, sample->mag);
	}

	for (int i = 0; i < 3; i++) {
		float gyro = rate_b[i] + sim_true_bias[i] + SIM_GYRO_NOISE * sim_randn(&sim->seed);

		if (sim->gyro_limit > 0.0f) {
			gyro = fminf(sim->gyro_limit, fmaxf(-sim->gyro_limit, gyro));
		}
		sample->gyro[i] = gyro;
		sample->accel[i] += SIM_ACCEL_NOISE * sim_randn(&sim->seed);
		sample->mag[i] += SIM_MAG_NOISE * sim_randn(&sim->seed);
	}
}
