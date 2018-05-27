package org.usfirst.frc.team1257.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {
	XboxController DriveController = new XboxController(0);
	
	WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(2);
	SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

	WPI_TalonSRX m_frontRight = new WPI_TalonSRX(3);
	WPI_TalonSRX m_rearRight = new WPI_TalonSRX(4);
	SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

	DifferentialDrive DriveTrain = new DifferentialDrive(m_left, m_right);
	
	@Override
	public void robotInit() {

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	
	
	
	@Override
	public void teleopPeriodic() {
		Drive();
	}
	
	public void Drive(){
		double forwardSpeed = 0;
		double turnSpeed = 0;
		
		// If they press A, use single stick arcade with the left joystick
		if(DriveController.getAButton())
		{
			forwardSpeed = DriveController.getX(GenericHID.Hand.kLeft);
			turnSpeed = DriveController.getX(GenericHID.Hand.kLeft);
		}
		// If they press the left bumper, use the left joystick for forward and
		// backward motion and the right joystick for turning
		else if(DriveController.getBumper(GenericHID.Hand.kLeft))
		{
			forwardSpeed = DriveController.getY(GenericHID.Hand.kLeft);
			turnSpeed = DriveController.getX(GenericHID.Hand.kRight);
		}
		// If they press the right bumper, use the right joystick for forward and
		// backward motion and the left joystick for turning
		else if(DriveController.getBumper(GenericHID.Hand.kRight))
		{
			forwardSpeed = DriveController.getY(GenericHID.Hand.kRight);
			turnSpeed = DriveController.getX(GenericHID.Hand.kLeft);
		}

		// Negative is used to make forward positive and backwards negative
		// because the y-axes of the XboxController are natively inverted
		DriveTrain.arcadeDrive(-forwardSpeed, turnSpeed);
	}
	

	
	@Override
	public void testPeriodic() {
	}
}
