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

	AHRS Ahrs;
	DifferentialDrive DriveTrain;
	XboxController DriveController;

	Timer timer;
	double lastAccTime;
	double lastJerkTime;

public:
	Robot() :
		BackRightMotor(0),
		FrontRightMotor(1),
		FrontLeftMotor(2),
		BackLeftMotor(3),
		Ahrs(SPI::kOnboardCS0),
		DriveTrain(FrontLeftMotor, FrontRightMotor),
		DriveController(0),
		timer(),
		lastAccTime(0),
		lastJerkTime(0)
	{

	}
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

	void Drive();
	void LogData();
};
