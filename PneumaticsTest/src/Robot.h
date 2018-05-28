#pragma once

#include <WPILib.h>
#include "EnhancedDoubleSolenoid.h"

using namespace frc;

class Robot : public IterativeRobot
{
private:
	XboxController DriveController;
	EnhancedDoubleSolenoid left;
	EnhancedDoubleSolenoid right;

public:
	Robot();
	void RobotInit() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
};
