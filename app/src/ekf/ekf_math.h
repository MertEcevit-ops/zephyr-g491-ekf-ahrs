/*
 * Quaternion and attitude helpers for the EKF.
 *
 * Conventions (MATH_SPEC.md section 1): Hamilton quaternions, scalar first,
 * q rotates body-frame vectors into the NED navigation frame, Euler angles are
 * ZYX (yaw, pitch, roll). Angles in rad.
 */
#ifndef EKF_MATH_H_
#define EKF_MATH_H_

#include <stdbool.h>
#include <stddef.h>

#define EKF_PI 3.14159265358979f

/** out = p (x) r. out may alias neither input. */
void quat_multiply(const float p[4], const float r[4], float out[4]);

/**
 * Scale q to unit length.
 *
 * @param norm_before receives the length q had on entry
 * @retval -EDOM the length is too small to normalise; q is left untouched
 */
int quat_normalize(float q[4], float *norm_before);

/** Exact quaternion of a rotation vector (angle * axis), series near zero. */
void quat_from_rotation_vector(const float rotvec[3], float dq[4]);

/** Body-to-navigation rotation matrix R(q), row-major. */
void quat_to_rotation_matrix(const float q[4], float rot[3][3]);

void quat_from_euler(float roll, float pitch, float yaw, float q[4]);
void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);

/** q (x) [0, w] = quat_omega(w) q. */
void quat_omega_matrix(const float w[3], float omega[4][4]);

/** q (x) [0, w] = quat_xi(q) w. The columns are orthogonal to q. */
void quat_xi_matrix(const float q[4], float xi[4][3]);

/**
 * Predicted specific-force direction for a body at rest and its Jacobian with
 * respect to q (MATH_SPEC.md section 8).
 */
void ekf_gravity_direction(const float q[4], float predicted[3], float jacobian[3][4]);

/**
 * Jacobian of yaw with respect to q (MATH_SPEC.md section 9).
 *
 * @param cos2_pitch receives cos^2(pitch); the Jacobian is singular as it
 *                   approaches zero and the caller must gate on it
 */
void ekf_heading_jacobian(const float q[4], float jacobian[4], float *cos2_pitch);

/** Wrap an angle into (-pi, pi]. */
float ekf_wrap_pi(float angle);

float vec3_norm(const float v[3]);
bool ekf_all_finite(const float *values, size_t count);

#endif /* EKF_MATH_H_ */
