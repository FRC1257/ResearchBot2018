package org.usfirst.frc.team1257.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.usfirst.frc.team1257.robot.subsystems.DriveTrainSubsystem;

public class Robot extends TimedRobot {
	public static DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();
	public static OI m_oi;
	RobotMap m_robotMap = new RobotMap();

	
	
	@Override
	public void robotInit() {
		m_oi = new OI();
	}
	
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}
	
	

	@Override
	public void autonomousInit() {
		
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}
	

	
	@Override
	public void teleopInit() {

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
	
	

	@Override
	public void testPeriodic() {
		
	}
}
