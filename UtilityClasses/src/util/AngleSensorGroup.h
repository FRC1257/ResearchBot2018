#ifndef ANGLE_SENSOR_GROUP
#define ANGLE_SENSOR_GROUP

#include <wpilib.h>
#include "AHRS.h"

class AngleSensorGroup : public PIDSource
{
private:
	AHRS m_NavX;
	ADXRS450_Gyro m_Gyro;

public:
	AngleSensorGroup(SPI::Port navXPort, SPI::Port gyroPort) :
		m_NavX(navXPort),
		m_Gyro(gyroPort)
	{

	}
	virtual ~AngleSensorGroup()
	{

	}

	void Reset()
	{
		m_NavX.ZeroYaw();
		m_Gyro.Reset();
	}

	double GetAngle()
	{
		if(m_NavX.IsConnected())
		{
			return m_NavX.GetYaw();
		}
		else
		{
			return m_Gyro.GetAngle();
		}
	}

	double PIDGet()
	{
		return GetAngle();
	}
};

#endif
