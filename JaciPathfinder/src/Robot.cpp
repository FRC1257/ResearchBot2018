#include "Robot.h"

void Robot::RobotInit()
{
	Waypoint p1 = { 0, 0, 0 };
	Waypoint p2 = { 0, 1, 0 };
	Waypoint p3 = { 0, 3, 0 };

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIME_STEP,
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
			PATH_P, PATH_I, PATH_D, PATH_F, PATH_A
	};

	free(trajectory);

	BackRightMotor.Follow(FrontRightMotor);
	BackLeftMotor.Follow(FrontLeftMotor);

	FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, PID_LOOP_ID, TALON_TIMEOUT_MS);
	FrontLeftMotor.SetSensorPhase(true);

	FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, PID_LOOP_ID, TALON_TIMEOUT_MS);
	FrontRightMotor.SetSensorPhase(true);


	SmartDashboard::PutNumber("Left Encoder Vel", 0);
	SmartDashboard::PutNumber("Right Encoder Vel", 0);

	SmartDashboard::PutNumber("Left Encoder Acc", 0);
	SmartDashboard::PutNumber("Right Encoder Acc", 0);
}

void Robot::AutonomousInit()
{

}

void Robot::AutonomousPeriodic()
{
	double l = pathfinder_follow_encoder(config, &leftFollower, leftTrajectory, POINT_LENGTH, FrontLeftMotor.GetSelectedSensorPosition(0));
	double r = pathfinder_follow_encoder(config, &rightFollower, rightTrajectory, POINT_LENGTH, FrontRightMotor.GetSelectedSensorPosition(0));

	double gyroHeading = Ahrs.GetYaw();
	double desired = r2d(leftFollower.heading);

	double angleDifference = desired - gyroHeading;
	double turn = ANGLE_P * angleDifference;

	FrontLeftMotor.Set(l + turn);
	FrontRightMotor.Set(r - turn);
}

void Robot::TeleopInit()
{
	timer.Start();
}

void Robot::TeleopPeriodic()
{
	Drive();
	LogData();
}

void Robot::Drive()
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
}

void Robot::LogData()
{
	SmartDashboard::PutNumber("Left Encoder Pos", InchesToMeters(PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0))));
	SmartDashboard::PutNumber("Right Encoder Pos", InchesToMeters(PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0))));

	double currentLeftVel = InchesToMeters(PulsesToInches(FrontLeftMotor.GetSelectedSensorVelocity(0))) * 10;
	double currentRightVel = InchesToMeters(PulsesToInches(FrontRightMotor.GetSelectedSensorVelocity(0))) * 10;

	double currentLeftVelDiff = currentLeftVel - SmartDashboard::GetNumber("Left Encoder Vel", 0);
	double currentRightVelDiff = currentRightVel - SmartDashboard::GetNumber("Right Encoder Vel", 0);

	SmartDashboard::PutNumber("Left Encoder Vel", currentLeftVel);
	SmartDashboard::PutNumber("Right Encoder Vel", currentRightVel);

	double currentLeftAcc = currentLeftVelDiff / (timer.Get() - lastAccTime);
	double currentRightAcc = currentLeftVelDiff / (timer.Get() - lastAccTime);

	lastAccTime = timer.Get();

	double currentLeftAccDiff = currentLeftAcc - SmartDashboard::GetNumber("Left Encoder Vel", 0);
	double currentRightAccDiff = currentRightAcc - SmartDashboard::GetNumber("Right Encoder Vel", 0);

	SmartDashboard::PutNumber("Left Encoder Acc", currentLeftAcc);
	SmartDashboard::PutNumber("Right Encoder Acc", currentRightAcc);

	double currentLeftJerk = currentLeftAccDiff / (timer.Get() - lastJerkTime);
	double currentRightJerk = currentRightAccDiff / (timer.Get() - lastJerkTime);

	lastJerkTime = timer.Get();
}

void Robot::TestPeriodic()
{

}

START_ROBOT_CLASS(Robot)
