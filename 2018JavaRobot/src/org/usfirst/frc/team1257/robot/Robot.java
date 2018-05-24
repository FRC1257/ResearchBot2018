/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1257.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	XboxController DriveController = new XboxController(0);
	WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(2);
	SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

	WPI_TalonSRX m_frontRight = new WPI_TalonSRX(3);
	WPI_TalonSRX m_rearRight = new WPI_TalonSRX(4);
	SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

	DifferentialDrive DriveTrain = new DifferentialDrive(m_left, m_right);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
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
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
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
			turnSpeed = DriveController.getX(GenericHID.Hand.kRight);
		}

		// Negative is used to make forward positive and backwards negative
		// because the y-axes of the XboxController are natively inverted
		DriveTrain.arcadeDrive(-forwardSpeed, turnSpeed);
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
