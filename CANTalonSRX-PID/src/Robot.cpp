#include "WPILib.h"
#include <ctre/Phoenix.h>
#include "Constants.h"

using namespace frc;

double PulsesToInches(double sensorPosition)
{
	double circumference = consts::WHEEL_DIAMETER * consts::PI;
	double revolutions = sensorPosition / consts::PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

double InchesToPulses(double inches)
{
	double circumference = consts::WHEEL_DIAMETER * consts::PI;
	double revolutions = inches / circumference;
	double pulses = revolutions * consts::PULSES_PER_REV;

	return pulses;
}

double ConvertAngleToDistance(double angle, double distanceBetweenWheels)
{
	return InchesToPulses(consts::PI * distanceBetweenWheels * angle / 360.0);
}

class Robot : public IterativeRobot
{
private:
	WPI_TalonSRX BackRightMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;

	DifferentialDrive DriveTrain;
	XboxController DriveController;

	bool lastButton;

public:
	Robot() :
		BackRightMotor(1),
		FrontRightMotor(2),
		FrontLeftMotor(3),
		BackLeftMotor(4),

		DriveTrain(FrontLeftMotor, FrontRightMotor),
		DriveController(0),
		lastButton(false)
	{

	}

	void RobotInit() override
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

		FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontLeftMotor.SetSensorPhase(true);

		FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSensorPhase(true);

		FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);

		FrontLeftMotor.ConfigAllowableClosedloopError(0, InchesToPulses(2), consts::TALON_TIMEOUT_MS);
		FrontRightMotor.ConfigAllowableClosedloopError(0, InchesToPulses(2), consts::TALON_TIMEOUT_MS);

		FrontLeftMotor.SelectProfileSlot(0, 0);
		FrontLeftMotor.Config_kP(0, 0.2, consts::PID_TIMEOUT_S);
		FrontLeftMotor.Config_kI(0, 0, consts::PID_TIMEOUT_S);
		FrontLeftMotor.Config_kD(0, 0, consts::PID_TIMEOUT_S);
		FrontLeftMotor.Config_kF(0, 0, consts::PID_TIMEOUT_S);

		FrontRightMotor.SelectProfileSlot(0, 0);
		FrontRightMotor.Config_kP(0, 0.2, consts::PID_TIMEOUT_S);
		FrontRightMotor.Config_kI(0, 0, consts::PID_TIMEOUT_S);
		FrontRightMotor.Config_kD(0, 0, consts::PID_TIMEOUT_S);
		FrontRightMotor.Config_kF(0, 0, consts::PID_TIMEOUT_S);

//		FrontLeftMotor.Follow(FrontRightMotor);
		BackLeftMotor.Follow(FrontLeftMotor);
		BackRightMotor.Follow(FrontRightMotor);

		SmartDashboard::PutBoolean("Reset Encoders", false);
	}

	void AutonomousInit() override
	{
		FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);

		DriveDistance(100);
	}

	void DriveDistance(double distance)
	{
		distance = InchesToPulses(distance);
		FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);

		FrontLeftMotor.Set(ControlMode::Position, distance);
		FrontRightMotor.Set(ControlMode::Position, -distance);

		SmartDashboard::PutNumber("Error L", PulsesToInches(
				std::abs(FrontLeftMotor.GetClosedLoopError(consts::PID_LOOP_ID))));
		SmartDashboard::PutNumber("Error R", PulsesToInches(
				std::abs(FrontRightMotor.GetClosedLoopError(consts::PID_LOOP_ID))));

		while(std::abs(FrontLeftMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(3) ||
				std::abs(FrontRightMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(3))
		{
			SmartDashboard::PutNumber("Error L", PulsesToInches(
					std::abs(FrontLeftMotor.GetClosedLoopError(consts::PID_LOOP_ID))));
			SmartDashboard::PutNumber("Error R", PulsesToInches(
					std::abs(FrontRightMotor.GetClosedLoopError(consts::PID_LOOP_ID))));

			FrontLeftMotor.Set(ControlMode::Position, distance);
			FrontRightMotor.Set(ControlMode::Position, -distance);

			SmartDashboard::PutNumber("Left Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Right Distance", PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0)));
		}
	}

	void TurnAngle(double angle)
	{
		FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);

		double distance = ConvertAngleToDistance(angle, 20);

		FrontLeftMotor.Set(ControlMode::Position, distance);
		FrontRightMotor.Set(ControlMode::Position, -distance);

		while(std::abs(FrontLeftMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(2) ||
				std::abs(FrontRightMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(2))
		{
			FrontLeftMotor.Set(ControlMode::Position, distance);
			FrontRightMotor.Set(ControlMode::Position, -distance);

			SmartDashboard::PutNumber("Left Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Right Distance", PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0)));
		}
	}

	void AutonomousPeriodic() override
	{
		SmartDashboard::PutNumber("Left Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
		SmartDashboard::PutNumber("Right Distance", PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0)));
	}

	void TeleopInit() override
	{
		FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);

		if(FrontLeftMotor.GetControlMode() != ControlMode::PercentOutput) FrontLeftMotor.Set(ControlMode::PercentOutput, 0);
		if(FrontRightMotor.GetControlMode() != ControlMode::PercentOutput) FrontRightMotor.Set(ControlMode::PercentOutput, 0);
	}

	void TeleopPeriodic() override
	{
		SmartDashboard::PutNumber("Left Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
		SmartDashboard::PutNumber("Right Distance", PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0)));

		if(SmartDashboard::GetBoolean("Reset Encoders", false))
		{
			SmartDashboard::PutBoolean("Reset Encoders", false);
			FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
			FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		}

		if(true)
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
	}
};

START_ROBOT_CLASS(Robot)
