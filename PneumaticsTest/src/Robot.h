/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <WPILib.h>

using namespace frc;

class Robot : public IterativeRobot
{
private:
	XboxController DriveController;
	DoubleSolenoid left;
	DoubleSolenoid right;

public:
	Robot();
	void RobotInit() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
};
