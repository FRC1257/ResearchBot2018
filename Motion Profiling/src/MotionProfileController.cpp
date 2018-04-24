#include <MotionProfileController.h>
#include <cmath>
#include <algorithm>

MotionProfileController::MotionProfileController(MotionProfileConstants presets) :
	constants(presets),

	endPoint(0),
	startPoint(0),

	finished(false),
	started(false),

	maxVelocity(presets.maxVelocity),
	maxAcceleration(presets.maxAcceleration),

	travelDistance(0),
	adjustedMaxVelocity(maxVelocity),
	isTrapezoidal(false),

	backwards(false),
	acceleratingDistance(0),
	cruisingDistance(0),

	nextVelocity(0)
{

}

MotionProfileController::~MotionProfileController()
{

}

void MotionProfileController::SetSetpoint(double currentPosition, double setpoint)
{
	finished = false;
	started = true;
	endPoint = setpoint;
	nextVelocity = 0;
	startPoint = currentPosition;

	outputs[0] = currentPosition;
	outputs[1] = 0;
	outputs[2] = 0;

	backwards = currentPosition > setpoint;
	travelDistance = std::abs(setpoint - currentPosition);

	// Finds fastest possible velocity on path
	adjustedMaxVelocity = std::min(maxVelocity,
			std::sqrt(2 * maxAcceleration * (travelDistance / 2.0)));
	acceleratingDistance = (adjustedMaxVelocity * adjustedMaxVelocity) / (2 * maxAcceleration);

	// Trapezoidal vs Triangular profile
	isTrapezoidal = (acceleratingDistance < (travelDistance / 2.0));
	cruisingDistance = isTrapezoidal ? (travelDistance - (2 * acceleratingDistance)) : 0;
}

double MotionProfileController::Calculate(double currentPosition)
{
	if(!finished)
	{
		double distanceFromEnd = std::abs(endPoint - currentPosition);
		double distanceFromStart = std::abs(startPoint - currentPosition);

		double maxReachVelocity = (nextVelocity * nextVelocity) / 2.0 +
				(maxAcceleration * distanceFromEnd);
		double minReachVelocity = (nextVelocity * nextVelocity) / 2.0 -
				(maxAcceleration * distanceFromEnd);

		double newAdjustedMaxVelocity = nextVelocity;

		// Ensure velocity is non-negative
		if(!backwards)
		{
			if(minReachVelocity < 0 || newAdjustedMaxVelocity < 0)
			{
				newAdjustedMaxVelocity = std::min(adjustedMaxVelocity, std::sqrt(maxReachVelocity));
			}
		}
		else
		{
			if(minReachVelocity < 0)
			{
				newAdjustedMaxVelocity = std::min(adjustedMaxVelocity, std::sqrt(maxReachVelocity));
			}
		}

		// First leg of trapezoid/triangle
		if(distanceFromStart <= acceleratingDistance)
		{
			SetOutputArray(nextVelocity * DT + 0.5 * maxAcceleration * DT * DT,
		             nextVelocity + (maxAcceleration * DT),
		             maxAcceleration);
		}
		// Second leg of trapezoid/triangle
		else if(distanceFromStart > (acceleratingDistance + cruisingDistance))
		{
			SetOutputArray(newAdjustedMaxVelocity * DT - 0.5 * maxAcceleration * DT * DT,
		            newAdjustedMaxVelocity - (maxAcceleration * DT),
		            -maxAcceleration);
		}

		// Path is finished
		if(distanceFromStart >= travelDistance)
		{
			SetOutputArray(endPoint, 0, 0);
			finished = true;

			return 0.0;
		}

		// Cruising
		if(isTrapezoidal && (distanceFromStart <= (acceleratingDistance + cruisingDistance))
		         && (distanceFromStart > acceleratingDistance))
		{
			SetOutputArray(adjustedMaxVelocity * DT, adjustedMaxVelocity, 0);
		}

		return ((outputs[0] - currentPosition) * constants.kP) +
				(outputs[1] * constants.kF) +
				(outputs[2] * constants.kA);
	}

	return 0.0;
}

void MotionProfileController::SetOutputArray(double nextDistance, double nextVelocity,
		double nextAcceleration)
{
	outputs[0] += nextDistance * (backwards ? -1 : 1);
	outputs[1] += nextVelocity * (backwards ? -1 : 1);
	outputs[2] += nextAcceleration * (backwards ? -1 : 1);

	this->nextVelocity = nextVelocity;
}

