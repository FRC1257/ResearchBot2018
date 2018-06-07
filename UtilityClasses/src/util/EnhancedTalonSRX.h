#ifndef ENHANCED_TALON_SRX
#define ENHANCED_TALON_SRX

#include <WPILib.h>
#include <ctre/Phoenix.h>

#include "Constants.h"

using namespace frc;

class EnhancedTalonSRX : public WPI_TalonSRX
{
private:
	int timeoutMs = 10;
	int pidLoopId = 0;

	//For converting pulses to real world values
	double diameter = consts::WHEEL_DIAMETER;
	double pulsesPerRev = consts::PULSES_PER_REV;

public:
	EnhancedTalonSRX(int deviceNumber) :
		WPI_TalonSRX(deviceNumber)
	{

	}

	//Setters
	void SetTimeoutMs(int timeout)
	{
		timeoutMs = timeout;
	}
	void setPidLoopId(int pidLoopId)
	{
		this->pidLoopId = pidLoopId;
	}
	void setDiameter(double diameter)
	{
		this->diameter = diameter;
	}
	void setPulsesPerRev(double pulsePerRev)
	{
		this->pulsesPerRev = pulsesPerRev;
	}

	//Current Limiting
	void EnableCurrentLimit(int amps, int timeoutMs)
	{
		ConfigContinuousCurrentLimit(amps, timeoutMs);
		WPI_TalonSRX::EnableCurrentLimit(true);
	}
	void DisableCurrentLimit()
	{
		WPI_TalonSRX::EnableCurrentLimit(false);
	}

	//Encoder Values
	int GetSensorValue()
	{
		return WPI_TalonSRX::GetSelectedSensorPosition(pidLoopId);
	}
	void SetSensorValue(int value)
	{
		WPI_TalonSRX::SetSelectedSensorPosition(value, pidLoopId, timeoutMs);
	}
	void ZeroSensor()
	{
		SetSensorValue(0);
	}
	int GetSensorVelocity()
	{
		return WPI_TalonSRX::GetSelectedSensorVelocity(pidLoopId);
	}

	double GetSensorValueInches()
	{
		static double circumference = diameter * consts::PI;
		double revolutions = GetSensorValue() / pulsesPerRev;
		double distance = revolutions * circumference;

		return distance;
	}
	double GetSensorVelocityInchesPerSecond()
	{
		static double circumference = diameter * consts::PI;
		double revolutionsPerSecond = (GetSensorVelocity() * 10 / pulsesPerRev);
		double distancePerSecond = revolutionsPerSecond * circumference;

		return distancePerSecond;
	}

	void ConfigFeedbackSensor(FeedbackDevice feedbackDevice, bool sensorPhase)
	{
		WPI_TalonSRX::ConfigSelectedFeedbackSensor(feedbackDevice, pidLoopId, timeoutMs);
		WPI_TalonSRX::SetSensorPhase(sensorPhase);
	}
	void ConfigFeedbackSensor(RemoteFeedbackDevice feedbackDevice, bool sensorPhase)
	{
		WPI_TalonSRX::ConfigSelectedFeedbackSensor(feedbackDevice, pidLoopId, timeoutMs);
		WPI_TalonSRX::SetSensorPhase(sensorPhase);
	}

	//Limit switch
	void ConfigForwardLimitSwitchSource(LimitSwitchSource limitSwitchSource,
		LimitSwitchNormal normalOpenOrClose)
	{
		WPI_TalonSRX::ConfigForwardLimitSwitchSource(limitSwitchSource, normalOpenOrClose, timeoutMs);
	}
	void ConfigReverseLimitSwitchSource(LimitSwitchSource limitSwitchSource,
		LimitSwitchNormal normalOpenOrClose)
	{
		WPI_TalonSRX::ConfigReverseLimitSwitchSource(limitSwitchSource, normalOpenOrClose, timeoutMs);
	}
	void ConfigLimitSwitchSources(LimitSwitchSource forwardLimitSwitchSource, LimitSwitchNormal forwardNormalOpenOrClose,
			LimitSwitchSource reverseLimitSwitchSource, LimitSwitchNormal reverseNormalOpenOrClose)
	{
		ConfigForwardLimitSwitchSource(forwardLimitSwitchSource, forwardNormalOpenOrClose);
		ConfigReverseLimitSwitchSource(reverseLimitSwitchSource, reverseNormalOpenOrClose);
	}

	void ConfigForwardSoftLimitThresholdInches(double inches)
	{
			static double circumference = diameter * consts::PI;
			double revolutions = inches / circumference;
			double pulses = revolutions * pulsesPerRev;

			WPI_TalonSRX::ConfigForwardSoftLimitThreshold(pulses, timeoutMs);
	}
	void ConfigReverseSoftLimitThresholdInches(double inches)
	{
			static double circumference = diameter * consts::PI;
			double revolutions = inches / circumference;
			double pulses = revolutions * pulsesPerRev;

			WPI_TalonSRX::ConfigReverseSoftLimitThreshold(pulses, timeoutMs);
	}
	void ConfigSoftLimitThresholdsInches(double forwardInches, double reverseInches)
	{
		ConfigForwardSoftLimitThresholdInches(forwardInches);
		ConfigReverseSoftLimitThresholdInches(reverseInches);
	}

	void ConfigForwardSoftLimitEnable(bool enabled)
	{
		WPI_TalonSRX::ConfigForwardSoftLimitEnable(enabled, timeoutMs);
	}
	void ConfigReverseSoftLimitEnable(bool enabled)
	{
		WPI_TalonSRX::ConfigReverseSoftLimitEnable(enabled, timeoutMs);
	}
	void EnableSoftLimits(bool enabled)
	{
		ConfigForwardSoftLimitEnable(enabled);
		ConfigReverseSoftLimitEnable(enabled);
	}
};

#endif
