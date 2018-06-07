#ifndef ENHANCED_DOUBLE_SOLENOID
#define ENHANCED_DOUBLE_SOLENOID

#include <WPILib.h>

class EnhancedDoubleSolenoid : public DoubleSolenoid
{
public:
	EnhancedDoubleSolenoid(int forwardChannel, int reverseChannel) :
		DoubleSolenoid(forwardChannel, reverseChannel)
	{

	}

	void Set(Value value) override
	{
		if(DoubleSolenoid::Get() != value) DoubleSolenoid::Set(value);
	}
};

#endif
