/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <pathfinder.h>
#include "Consts.h"

using namespace frc;

inline double InchesToMeters(double inches)
{
	return inches * 0.0254;
}

inline double PulsesToInches(double sensorPosition)
{
	double circumference = WHEEL_DIAMETER * PI;
	double revolutions = sensorPosition / PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

class Robot : public TimedRobot
{
private:
	Waypoint points[POINT_LENGTH];
	TrajectoryCandidate candidate;
	Segment *trajectory;
	Segment *leftTrajectory;
	Segment *rightTrajectory;
	EncoderFollower leftFollower;
	EncoderFollower rightFollower;
	EncoderConfig config;

	WPI_TalonSRX BackRightMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;

	DifferentialDrive DriveTrain;
	XboxController DriveController;

	AHRS ahrs;

public:
	Robot() :
		BackRightMotor(0),
		FrontRightMotor(1),
		FrontLeftMotor(2),
		BackLeftMotor(3),
		ahrs(SPI::kOnboardCS0),
		DriveTrain(FrontLeftMotor, FrontRightMotor),
		DriveController(0)
	{

	}
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
};
