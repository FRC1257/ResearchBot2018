/*
 * Consts.h
 *
 *  Created on: May 16, 2018
 *      Author: Team1257
 */

#ifndef SRC_CONSTS_H_
#define SRC_CONSTS_H_

constexpr int POINT_LENGTH = 3;
constexpr double WHEELBASE_WIDTH = 0.6;
constexpr double WHEEL_DIAMETER = 0.1524;
constexpr int PULSES_PER_REV = 4096;

constexpr double MAX_VEL = 3;
constexpr double MAX_ACC = 0.8;
constexpr double MAX_JER = 3;

constexpr int PID_LOOP_ID = 0;
constexpr int TALON_TIMEOUT_MS = 10;

#endif /* SRC_CONSTS_H_ */
