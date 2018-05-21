/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit()
{
	Waypoint p1 = { 0, 0, 0 };
	Waypoint p2 = { 0, 1, 0 };
	Waypoint p3 = { 0, 3, 0 };

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001,
			MAX_VEL, MAX_ACC, MAX_JER, &candidate);
	int length = candidate.length;

	trajectory = (Segment*) (malloc(length * sizeof(Segment)));
	leftTrajectory = (Segment*) (malloc(length * sizeof(Segment)));
	rightTrajectory = (Segment*) (malloc(length * sizeof(Segment)));

	pathfinder_generate(&candidate, trajectory);

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, WHEELBASE_WIDTH);

	leftFollower = {0, 0, 0, 0, 0};
	rightFollower = {0, 0, 0, 0, 0};

	config = {
			0, PULSES_PER_REV, WHEEL_DIAMETER * PI,
			1.0, 0.0, 0.0, 1.0 / MAX_VEL, 0.0
	};

	free(trajectory);

	BackRightMotor.Follow(FrontRightMotor);
	BackLeftMotor.Follow(FrontLeftMotor);

	FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, PID_LOOP_ID, TALON_TIMEOUT_MS);
	FrontLeftMotor.SetSensorPhase(true);

	FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, PID_LOOP_ID, TALON_TIMEOUT_MS);
	FrontRightMotor.SetSensorPhase(true);
}

void Robot::AutonomousInit()
{

}

void Robot::AutonomousPeriodic()
{
	double l = pathfinder_follow_encoder(config, &leftFollower, leftTrajectory, POINT_LENGTH, FrontLeftMotor.GetSelectedSensorPosition(0));
	double r = pathfinder_follow_encoder(config, &rightFollower, rightTrajectory, POINT_LENGTH, FrontRightMotor.GetSelectedSensorPosition(0));

	double gyroHeading = ahrs.GetYaw();
	double desired = r2d(leftFollower.heading);

	double angleDifference = desired - gyroHeading;
	double turn = -0.01 * angleDifference;

	FrontLeftMotor.Set(l + turn);
	FrontRightMotor.Set(r - turn);
}

void Robot::TeleopInit()
{

}

void Robot::TeleopPeriodic()
{
	double forwardSpeed = 0;
	double turnSpeed = 0;

	// If they press A, use single stick arcade with the left joystick
	if(DriveController.GetAButton())
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}
	// If they press the left bumper, use the left joystick for forward and
	// backward motion and the right joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kRightHand);
	}
	// If they press the right bumper, use the right joystick for forward and
	// backward motion and the left joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kRightHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}

	// Negative is used to make forward positive and backwards negative
	// because the y-axes of the XboxController are natively inverted
	DriveTrain.ArcadeDrive(-forwardSpeed, turnSpeed);

	SmartDashboard::PutNumber("Left Encoder Pos", InchesToMeters(PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0))));
	SmartDashboard::PutNumber("Right Encoder Pos", InchesToMeters(PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0))));

	SmartDashboard::PutNumber("Left Encoder Vel", InchesToMeters(PulsesToInches(FrontLeftMotor.GetSelectedSensorVelocity(0))) * 10);
	SmartDashboard::PutNumber("Right Encoder Vel", InchesToMeters(PulsesToInches(FrontRightMotor.GetSelectedSensorVelocity(0))) * 10);


}

void Robot::TestPeriodic()
{

}

START_ROBOT_CLASS(Robot)
