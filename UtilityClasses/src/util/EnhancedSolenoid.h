#ifndef ENHANCED_DOUBLE_SOLENOID
#define ENHANCED_DOUBLE_SOLENOID

#include <WPILib.h>

class EnhancedDoubleSolenoid : public Solenoid
{
public:
	EnhancedDoubleSolenoid(int channel) :
		Solenoid(channel)
	{

	}

	void Set(bool on) override
	{
		if(Solenoid::Get() != on) Solenoid::Set(on);
	}
};

#endif
