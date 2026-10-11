/*
 * Telemetry stream on the UART shell backend and the `telemetry`, `mag` and
 * `ekf` shell commands. Record formats: TEST_PLAN.md section 4.0.
 */
#ifndef TELEMETRY_H_
#define TELEMETRY_H_

/** Telemetry thread entry; needs K_FP_REGS. */
void telemetry_run(void *p1, void *p2, void *p3);

#endif /* TELEMETRY_H_ */
