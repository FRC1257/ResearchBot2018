#ifndef MOTION_PROFILE
#define MOTION_PROFILE

struct MotionProfileConstants
{
	double kP; // Proportional
	double kF; // Velocity feed forward
	double kA; // Acceleration feed forward
	double maxVelocity;
	double maxAcceleration;
};

//Code originally written by FRC team 2590, adapted by FRC team 1257
class MotionProfileController
{
private:
	MotionProfileConstants constants;
	const double DT = 0.02; // In seconds

	// Path points
	double endPoint;
	double startPoint;

	// Outputs
	bool finished;
	bool started;
	double outputs[3];

	// Bounds
	double maxVelocity;
	double maxAcceleration;

	// Cruising
	double travelDistance;
	double adjustedMaxVelocity;
	bool isTrapezoidal;

	// Acceleration and Deceleration Distances
	bool backwards;
	double acceleratingDistance;
	double cruisingDistance;

	// Future Value
	double nextVelocity;

	void SetOutputArray(double nextDistance, double nextVelocity, double nextAcceleration);
public:
	MotionProfileController(MotionProfileConstants);
	virtual ~MotionProfileController();

	void ChangeGains(MotionProfileConstants newConstants)
	{
		constants = newConstants;
	}
	bool IsFinished()
	{
		return finished;
	}
	bool Started()
	{
		return started;
	}
	void SetStarted(bool newValue)
	{
		started = newValue;
	}

	// Sets the desired end value of the profile
	void SetSetpoint(double currentPosition, double setPoint);

	// Calculates the motor output to follow the profile
	double Calculate(double currentPosition);
};

#endif
