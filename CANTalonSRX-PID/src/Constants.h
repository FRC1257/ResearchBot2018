#ifndef CONSTANTS
#define CONSTANTS

namespace consts
{
	// Talon configuration constants
	constexpr int PID_LOOP_ID = 0;
	constexpr int TALON_TIMEOUT_MS = 10;

	constexpr double TALON_P = 0.2;
	constexpr double TALON_I = 0.0;
	constexpr double TALON_D = 0.0;
	constexpr double TALON_F = 0.0;

	constexpr double DISTANCE_BETWEEN_WHEELS = 20;

	constexpr double TALON_TOLERANCE = 2;

	// Current Limiting Constants
	constexpr int FORTY_AMP_FUSE_CONT_MAX = 50; // The continuous max current draw for a 40 amp breaker
	constexpr int THIRTY_AMP_FUSE_CONT_MAX = 35; // The continuous max current draw for a 30 amp breaker
	constexpr int CONT_CURRENT_TIMEOUT_MS = 500;

	// Encoder Constants
	constexpr double PI = 3.1416;
	constexpr double WHEEL_DIAMETER = 6;
	constexpr double PULSES_PER_REV = 4096;
}

#endif
