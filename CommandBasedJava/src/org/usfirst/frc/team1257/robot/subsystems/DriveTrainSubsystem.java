package org.usfirst.frc.team1257.robot.subsystems;

import org.usfirst.frc.team1257.robot.RobotMap;
import org.usfirst.frc.team1257.robot.commands.DriveCommand;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */

public class DriveTrainSubsystem extends Subsystem {
	
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	Spark m_frontLeft = RobotMap.m_frontLeft;
	Spark m_rearLeft = RobotMap.m_rearLeft;
	SpeedControllerGroup m_left = RobotMap.m_left;

	Spark m_frontRight = RobotMap.m_frontRight;
	Spark m_rearRight = RobotMap.m_rearRight;
	SpeedControllerGroup m_right = RobotMap.m_right;

	public DifferentialDrive DriveTrain = RobotMap.DriveTrain;
    

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveCommand());
    }
}

