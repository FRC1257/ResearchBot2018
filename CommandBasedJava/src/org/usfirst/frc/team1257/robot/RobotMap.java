/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1257.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
//	private static final String kDefaultAuto = "Default";
//	private static final String kCustomAuto = "My Auto";
//	private String m_autoSelected;
//	private SendableChooser<String> m_chooser = new SendableChooser<>();

	public static WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(3);
	public static WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(4);
	public static SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

	public static WPI_TalonSRX m_frontRight = new WPI_TalonSRX(2);
	public static WPI_TalonSRX m_rearRight = new WPI_TalonSRX(1);
	public static SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

	public static DifferentialDrive DriveTrain = new DifferentialDrive(m_left, m_right);
}
