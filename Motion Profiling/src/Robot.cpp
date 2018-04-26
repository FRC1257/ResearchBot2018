#include <WPILib.h>
#include <ctre/Phoenix.h>
#include "Constants.h"
#include "MotionProfileController.h"

using namespace frc;

double PulsesToInches(double sensorPosition)
{
	double circumference = consts::WHEEL_DIAMETER * consts::PI;
	double revolutions = sensorPosition / consts::PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

double median(std::deque<double> array)
{
	// Can't use a reference because 'nth_element' would reorder the
	// original array
	size_t midPoint = array.size() / 2;
	nth_element(array.begin(), array.begin() + midPoint, array.end());
	return array[midPoint];
}

class Robot : public TimedRobot
{
private:

	WPI_TalonSRX BackRightMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;

	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	DifferentialDrive DriveTrain;
	XboxController DriveController;

	MotionProfileConstants constants;
	MotionProfileController controller;

	double lastPosition;
	double lastVelocity;

	double maxVel;
	double maxAcc;

	std::deque<double> accArray;


public:
	Robot() :
		BackRightMotor(1),
		FrontRightMotor(2),
		FrontLeftMotor(3),
		BackLeftMotor(4),

		LeftMotors(FrontLeftMotor, BackLeftMotor),
		RightMotors(FrontRightMotor, BackRightMotor),
		DriveTrain(LeftMotors, RightMotors),
		DriveController(0),

		constants{0, 0.005, 0, 130, 35},
		controller(constants),

		lastPosition(0),
		lastVelocity(0),

		maxVel(0),
		maxAcc(0),

		accArray()
	{

	}

	void RobotInit()
	{
		// Current limiting
		FrontLeftMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
		FrontLeftMotor.EnableCurrentLimit(true);

		FrontRightMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
		FrontRightMotor.EnableCurrentLimit(true);

		BackLeftMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
		BackLeftMotor.EnableCurrentLimit(true);

		BackRightMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
		BackRightMotor.EnableCurrentLimit(true);

		// Configuring the Talon Drive Encoders
		FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontLeftMotor.SetSensorPhase(true);

		FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSensorPhase(true);

		SmartDashboard::PutNumber("kP", 0);
		SmartDashboard::PutNumber("kF", 0);
		SmartDashboard::PutNumber("kA", 0);
		SmartDashboard::PutNumber("Max Velocity", 0);
		SmartDashboard::PutNumber("Max Acceleration", 0);

		SmartDashboard::PutBoolean("Reset Encoders", false);
		SmartDashboard::PutBoolean("Change Gains", false);

		FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		if(SmartDashboard::GetBoolean("Reset Encoders", false))
		{
			FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
			FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
			SmartDashboard::PutBoolean("Reset Encoders", false);
		}
//		if(SmartDashboard::GetBoolean("Change Gains", false))
//		{
//			MotionProfileConstants newConstants = {SmartDashboard::GetNumber("kP", 0),
//					SmartDashboard::GetNumber("kF", 0),
//					SmartDashboard::GetNumber("kV", 0),
//					SmartDashboard::GetNumber("Max Velocity", 0),
//					SmartDashboard::GetNumber("Max Acceleration", 0)};
//			controller.ChangeGains(newConstants);
//			SmartDashboard::PutBoolean("Change Gains", false);
//		}

		if(DriveController.GetYButton())
		{
			if(!controller.Started())
			{
				controller.SetSetpoint(PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)), 100);
			}
//			if(!controller.IsFinished())
			{
				double output = controller.Calculate(PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
				DriveTrain.ArcadeDrive(output, 0, false);
			}
		}
		else
		{
			controller.SetStarted(false);

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

			double robotPosition = PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("Robot Position", robotPosition);

			double posDiff = robotPosition - lastPosition;
			double velocity = posDiff / 0.02;
			SmartDashboard::PutNumber("Robot Velocity", velocity);
			double velDiff = velocity - lastVelocity;
			double acceleration = velDiff / 0.02;
			SmartDashboard::PutNumber("Robot Acceleration", velDiff / 0.020);

			lastPosition = robotPosition;
			lastVelocity = velocity;

			if(velocity > maxVel) maxVel = velocity;
			if(acceleration > maxAcc) maxAcc = acceleration;

			if(acceleration > 0) accArray.push_back(acceleration);

			SmartDashboard::PutNumber("Max Vel", maxVel);
			SmartDashboard::PutNumber("Max Acc", maxAcc);
			SmartDashboard::PutNumber("Median Acc", median(accArray));
		}
	}
};

START_ROBOT_CLASS(Robot)
