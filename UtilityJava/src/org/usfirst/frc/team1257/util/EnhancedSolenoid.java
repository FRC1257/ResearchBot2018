package org.usfirst.frc.team1257.util;

import edu.wpi.first.wpilibj.Solenoid;

public class EnhancedSolenoid extends Solenoid 
{
	
	public EnhancedSolenoid(int channel)
	{
		super(channel);
	}
	
	@Override
	public void set(boolean on)
	{
		if(get() != on) super.set(on);
	}
}
