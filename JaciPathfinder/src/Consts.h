#ifndef SRC_CONSTS_H_
#define SRC_CONSTS_H_

constexpr int POINT_LENGTH = 3;
constexpr double WHEELBASE_WIDTH = 0.6;
constexpr double WHEEL_DIAMETER = 0.1524;
constexpr int PULSES_PER_REV = 4096;

constexpr double MAX_VEL = 3;
constexpr double MAX_ACC = 0.8;
constexpr double MAX_JER = 3;

constexpr double PATH_P = 1.0;
constexpr double PATH_I = 0.0;
constexpr double PATH_D = 0.0;
constexpr double PATH_F = 1.0 / MAX_VEL;
constexpr double PATH_A = 0.0;

constexpr double ANGLE_P = -0.01;

constexpr double TIME_STEP = 0.02;

constexpr int PID_LOOP_ID = 0;
constexpr int TALON_TIMEOUT_MS = 10;

#endif /* SRC_CONSTS_H_ */
