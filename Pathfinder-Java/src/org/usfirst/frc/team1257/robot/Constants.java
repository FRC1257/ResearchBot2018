package org.usfirst.frc.team1257.robot;

public class Constants {

	public static double deadband(double value) {
		return Math.abs(value) < 0.08 ? 0 : value;
	}
	
	public static double inchesToMeters(double inches) {
		return inches * 0.0254;
	}

	public static class ElectricLayout {
		public static final int DRIVE_FRONT_LEFT = 0;
		public static final int DRIVE_FRONT_RIGHT = 1;
		public static final int DRIVE_BACK_LEFT = 2;
		public static final int DRIVE_BACK_RIGHT = 3;
	}
	
	//All in meters
	public static final double MAX_VELOCITY = 10;
	public static final double MAX_ACCELERATION = 10;
	public static final double MAX_JERK = 10;
	
	public static final double WHEEL_BASE_WIDTH_M = 0.6;
	
	public static final double LEFT_P = 1.0;
	public static final double LEFT_I = 0.0;
	public static final double LEFT_D = 0.0;
	public static final double LEFT_V = 1.0 / MAX_VELOCITY;
	public static final double LEFT_A = 0.0;

	public static final double RIGHT_P = 1.0;
	public static final double RIGHT_I = 0.0;
	public static final double RIGHT_D = 0.0;
	public static final double RIGHT_V = 1.0 / MAX_VELOCITY;
	public static final double RIGHT_A = 0.0;

	// Talon configuration constants
	public static final int PID_LOOP_ID = 0;
	public static final int TALON_TIMEOUT_MS = 10;

	// Current Limiting Constants
	public static final int FORTY_AMP_FUSE_CONT_MAX = 50; // The continuous max current draw for a 40 amp breaker
	public static final int THIRTY_AMP_FUSE_CONT_MAX = 35; // The continuous max current draw for a 30 amp breaker
	public static final int CONT_CURRENT_TIMEOUT_MS = 500;

	// Encoder Constants
	public static final double PI = 3.1416;
	public static final double WHEEL_DIAMETER = 6;
	public static final double PULSES_PER_REV = 4096;
}
