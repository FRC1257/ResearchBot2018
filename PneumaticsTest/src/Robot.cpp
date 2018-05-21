/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

Robot::Robot() :
	DriveController(0),
	left(0, 1),
	right(2, 3)
{

}

void Robot::RobotInit()
{

}

void Robot::TeleopInit()
{

}

void Robot::TeleopPeriodic()
{
	if(DriveController.GetAButton())
	{
		right.Set(DoubleSolenoid::Value::kForward);
	}
	else if(DriveController.GetBButton())
	{
		right.Set(DoubleSolenoid::Value::kReverse);
	}
	else if(!DriveController.GetBumper(GenericHID::kLeftHand) && !DriveController.GetBumper(GenericHID::kRightHand))
	{
		right.Set(DoubleSolenoid::Value::kOff);
	}

	if(DriveController.GetXButton())
	{
		left.Set(DoubleSolenoid::Value::kForward);
	}
	else if(DriveController.GetYButton())
	{
		left.Set(DoubleSolenoid::Value::kReverse);
	}
	else if(!DriveController.GetBumper(GenericHID::kLeftHand) && !DriveController.GetBumper(GenericHID::kRightHand))
	{
		left.Set(DoubleSolenoid::Value::kOff);
	}


	if(DriveController.GetBumper(GenericHID::kLeftHand))
	{
		left.Set(DoubleSolenoid::Value::kReverse);
		right.Set(DoubleSolenoid::Value::kReverse);
	}
	if(DriveController.GetBumper(GenericHID::kRightHand))
	{
		left.Set(DoubleSolenoid::Value::kForward);
		right.Set(DoubleSolenoid::Value::kForward);
	}
}

START_ROBOT_CLASS(Robot)
