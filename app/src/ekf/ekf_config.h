/*
 * Filter constants. Every value mirrors MATH_SPEC.md section 14; change the
 * table there first.
 */
#ifndef EKF_CONFIG_H_
#define EKF_CONFIG_H_

#define EKF_G0 9.80665f

#define EKF_DT_NOMINAL 9.615385e-3f
#define EKF_DT_MIN     1.0e-4f
#define EKF_DT_MAX     0.05f

#define EKF_GYRO_ARW      1.0e-3f /* rad/s/sqrt(Hz) */
#define EKF_GYRO_BIAS_RW  2.0e-5f /* rad/s/sqrt(s) */
#define EKF_ACC_SIGMA     0.03f   /* unit-vector components */
#define EKF_HEADING_SIGMA 0.05f   /* rad */

#define EKF_P0_QUAT_SIGMA        0.05f
#define EKF_P0_BIAS_SIGMA_STATIC 1.745e-3f
#define EKF_P0_BIAS_SIGMA_MOVING 5.236e-2f

#define EKF_ACC_GATE_SOFT     0.05f
#define EKF_ACC_GATE_HARD     0.20f
#define EKF_ACC_R_INFLATE_MAX 100.0f
#define EKF_ACC_NIS_MAX       11.34f /* chi-square, 3 dof, 99 % */

#define EKF_MAG_NIS_MAX        6.63f /* chi-square, 1 dof, 99 % */
#define EKF_MAG_NORM_GATE      0.15f
#define EKF_MAG_DIP_GATE       0.17453f
#define EKF_MAG_MIN_COS2_PITCH 0.0302f

/* Sample counts below assume the 104 Hz output data rate. */
#define EKF_YAW_RESET_REJECTS    312U
#define EKF_MAG_CONVERGE_ACCEPTS 104U
#define EKF_MAG_DISTRUST_REJECTS 3120U
#define EKF_TILT_RESET_REJECTS   520U

#define EKF_GYRO_SAT 8.29f /* 95 % of the +-500 dps range */

#define EKF_ALIGN_SAMPLES      100U
#define EKF_ALIGN_MAG_MIN      50U
#define EKF_ALIGN_GYRO_STD_MAX 0.02f
#define EKF_ALIGN_ACC_NORM_TOL 0.05f
#define EKF_ALIGN_MAX_WINDOWS  10U

#define EKF_QEXP_SMALL_ANGLE 1.0e-3f

/* Input and numerical guards, MATH_SPEC.md section 11. */
#define EKF_QUAT_NORM_MIN  1.0e-6f
#define EKF_ACC_NORM_MIN   1.0e-3f
#define EKF_MAG_NORM_MIN   1.0e-6f
#define EKF_HEADING_S_MIN  1.0e-12f

#endif /* EKF_CONFIG_H_ */
