package org.usfirst.frc.team1257.util;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidControllerGroup {
	
	private Solenoid[] solenoids;
	
	public SolenoidControllerGroup(Solenoid solenoid, Solenoid... extraSolenoids)
	{
		solenoids = new Solenoid[solenoids.length + 1];
		solenoids[0] = solenoid;
		
		for(int i = 0; i < solenoids.length; i++)
		{
			solenoids[i + 1] = extraSolenoids[i];
		}
	}
	
	public void set(boolean value)
	{
		for(Solenoid solenoid : solenoids)
		{
			solenoid.set(value);
		}
	}
}
