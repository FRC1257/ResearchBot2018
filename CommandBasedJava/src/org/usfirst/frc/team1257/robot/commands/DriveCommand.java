package org.usfirst.frc.team1257.robot.commands;

import org.usfirst.frc.team1257.robot.OI;
import org.usfirst.frc.team1257.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCommand extends Command {

    public DriveCommand() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.m_driveTrainSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double forwardSpeed = 0;
    	double turnSpeed = 0;

		if(OI.DriveController.getAButton())
		{
			forwardSpeed = OI.DriveController.getY(GenericHID.Hand.kLeft);
			turnSpeed = OI.DriveController.getX(GenericHID.Hand.kLeft);
		}
		// If they press the left bumper, use the left joystick for forward and
		// backward motion and the right joystick for turning
		else if(OI.DriveController.getBumper(GenericHID.Hand.kLeft))
		{
			forwardSpeed = OI.DriveController.getY(GenericHID.Hand.kLeft);
			turnSpeed = OI.DriveController.getX(GenericHID.Hand.kRight);
		}
		// If they press the right bumper, use the right joystick for forward and
		// backward motion and the left joystick for turning
		else if(OI.DriveController.getBumper(GenericHID.Hand.kRight))
		{
			forwardSpeed = OI.DriveController.getY(GenericHID.Hand.kRight);
			turnSpeed = OI.DriveController.getX(GenericHID.Hand.kLeft);
		}
		
    	Robot.m_driveTrainSubsystem.DriveTrain.arcadeDrive(-forwardSpeed, turnSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.m_driveTrainSubsystem.DriveTrain.arcadeDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
}
