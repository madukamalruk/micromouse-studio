/*
 * common — robot constants shared by every demo_* solver. On a real robot these come from the build
 * of that robot and its calibration runs; here they match config/robots/generic_classic.json.
 * Units: mm, s, degrees, volts.
 */
#ifndef FWC_CONFIG_H
#define FWC_CONFIG_H

/* ---- Maze ------------------------------------------------------------------------------ */
#define MAZE_SIZE 16
#define CELL_MM 180.0f
#define HALF_CELL_MM 90.0f

/* ---- Mechanics ------------------------------------------------------------------------- */
#define WHEEL_DIAMETER_MM 32.0f
#define GEAR_RATIO 30.0f
#define ENCODER_CPR_MOTOR 28.0f /* quadrature counts per motor revolution */
#define MM_PER_COUNT (3.14159265f * WHEEL_DIAMETER_MM / (ENCODER_CPR_MOTOR * GEAR_RATIO))
#define TRACK_MM 72.0f
#define DEG_TO_RAD 0.01745329f

/* ---- Motor safety (N20 on 2S LiPo) ------------------------------------------------------
 * Terminal voltage is capped at 6.0 V: duty = V / V_battery, never 100 % PWM.
 * A change of direction always passes through >= 20 ms of coasting (0 V). */
#define MOTOR_MAX_VOLTS 6.0f
#define MOTOR_REVERSAL_COAST_MS 20
#define MOTOR_DEADBAND_VOLTS 0.02f

/* ---- Feed-forward (per wheel; from the motor model, see docs/decisions.md D8) ------------ */
#define KV_WHEEL 0.00438f /* V per mm/s of wheel speed (back-EMF + viscous) */
#define KA_WHEEL 0.00024f /* V per mm/s^2 of wheel acceleration */
#define KS_WHEEL 0.10f    /* V to break static friction, in the direction of motion */

/* ---- Feedback (errors accumulated per 1 ms tick) ---------------------------------------- */
#define FWD_KP 0.10f /* V per mm */
#define FWD_KD 1.00f /* V per mm/tick */
#define ROT_KP 0.05f /* V per degree */
#define ROT_KD 1.00f /* V per degree/tick */

/* ---- Distance sensors (front_diagonal IR layout) ---------------------------------------- */
#define SENSOR_LEFT 0                 /* diagonal, 45 deg left */
#define SENSOR_FRONT_LEFT 1           /* forward */
#define SENSOR_FRONT_RIGHT 2          /* forward */
#define SENSOR_RIGHT 3                /* diagonal, 45 deg right */
#define DIAG_NOMINAL_MM 109.0f        /* diagonal reading when centred between two walls */
#define SIDE_WALL_MM 140.0f           /* diagonal below this = wall beside the cell being entered */
#define FRONT_WALL_BOUNDARY_MM 170.0f /* front below this at a cell boundary = wall ahead */
#define FRONT_NEAR_MM 80.0f           /* front wall close: diagonals see it, stop steering */
#define FRONT_ALIGN_MM 34.0f          /* front reading with the axle at the cell centre */
#define FRONT_ALIGN_GAP_MM 50.0f      /* lateral distance between the two front sensors */

/* ---- Wall-following steering (adds to the rotation set-point) --------------------------- */
#define STEER_KP 1.0f /* deg/s per mm of diagonal error */
#define STEER_MAX_DPS 60.0f
#define STEER_MIN_SPEED 100.0f /* mm/s */

/* ---- Speeds ---------------------------------------------------------------------------- */
#define SEARCH_SPEED 700.0f
#define SEARCH_ACCEL 3500.0f
#define RUN_SPEED 1200.0f
#define RUN_ACCEL 4000.0f
#define TURN_SPEED 720.0f  /* deg/s */
#define TURN_ACCEL 9000.0f /* deg/s^2 */

#endif /* FWC_CONFIG_H */
