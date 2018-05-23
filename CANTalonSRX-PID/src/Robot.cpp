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

double ConvertAngleToDistance(double angle)
{
	return InchesToPulses(consts::PI * consts::DISTANCE_BETWEEN_WHEELS * angle / 360.0);
}

class Robot : public IterativeRobot
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

public:
	Robot() :
		BackRightMotor(1),
		FrontRightMotor(2),
		FrontLeftMotor(3),
		BackLeftMotor(4),

		LeftMotors(FrontLeftMotor, BackLeftMotor),
		RightMotors(FrontRightMotor, BackRightMotor),
		DriveTrain(LeftMotors, RightMotors),
		DriveController(0)
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

		FrontLeftMotor.ConfigAllowableClosedloopError(0, InchesToPulses(consts::TALON_TOLERANCE), consts::TALON_TIMEOUT_MS);
		FrontRightMotor.ConfigAllowableClosedloopError(0, InchesToPulses(consts::TALON_TOLERANCE), consts::TALON_TIMEOUT_MS);

		FrontLeftMotor.SelectProfileSlot(0, 0);
		FrontLeftMotor.Config_kP(0, consts::TALON_P, consts::TALON_TIMEOUT_MS);
		FrontLeftMotor.Config_kI(0, consts::TALON_I, consts::TALON_TIMEOUT_MS);
		FrontLeftMotor.Config_kD(0, consts::TALON_D, consts::TALON_TIMEOUT_MS);
		FrontLeftMotor.Config_kF(0, consts::TALON_F, consts::TALON_TIMEOUT_MS);

		FrontRightMotor.SelectProfileSlot(0, 0);
		FrontRightMotor.Config_kP(0, consts::TALON_P, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.Config_kI(0, consts::TALON_I, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.Config_kD(0, consts::TALON_D, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.Config_kF(0, consts::TALON_F, consts::TALON_TIMEOUT_MS);

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

		while(std::abs(FrontLeftMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(consts::TALON_TOLERANCE) ||
				std::abs(FrontRightMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(consts::TALON_TOLERANCE))
		{
			FrontLeftMotor.Set(ControlMode::Position, distance);
			FrontRightMotor.Set(ControlMode::Position, distance);

			SmartDashboard::PutNumber("Left Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Right Distance", PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0)));
		}
	}

	void TurnAngle(double angle)
	{
		FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);

		double distance = ConvertAngleToDistance(angle);

		while(std::abs(FrontLeftMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(consts::TALON_TOLERANCE) ||
				std::abs(FrontRightMotor.GetClosedLoopError(consts::PID_LOOP_ID)) >= InchesToPulses(consts::TALON_TOLERANCE))
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

		if(FrontLeftMotor.GetControlMode() != ControlMode::PercentOutput) FrontLeftMotor.Set(ControlMode::PercentOutput, 0);
		if(FrontRightMotor.GetControlMode() != ControlMode::PercentOutput) FrontRightMotor.Set(ControlMode::PercentOutput, 0);

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
};

START_ROBOT_CLASS(Robot)
