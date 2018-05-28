#ifndef SRC_ENHANCEDDOUBLESOLENOID_H_
#define SRC_ENHANCEDDOUBLESOLENOID_H_

#include <WPILib.h>

using namespace frc;

class EnhancedDoubleSolenoid : DoubleSolenoid
{
public:
	EnhancedDoubleSolenoid(int forwardChannel, int reverseChannel) :
		DoubleSolenoid(forwardChannel, reverseChannel)
	{

	}

	void Set(Value value) override
	{
		if(value != DoubleSolenoid::Get()) DoubleSolenoid::Set(value);
	}
};

#endif
