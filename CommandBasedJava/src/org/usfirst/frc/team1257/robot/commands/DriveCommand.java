package org.usfirst.frc.team1257.robot.commands;

import org.usfirst.frc.team1257.robot.OI;
import org.usfirst.frc.team1257.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveCommand extends Command {

    public DriveCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.m_driveTrainSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double forwardSpeed = 0;
    	double turnSpeed = 0;
    	forwardSpeed = OI.DriveController.getX(GenericHID.Hand.kLeft);
		turnSpeed = OI.DriveController.getX(GenericHID.Hand.kLeft);
    	Robot.m_driveTrainSubsystem.DriveTrain.arcadeDrive(-forwardSpeed, turnSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
