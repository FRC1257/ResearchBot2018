package org.usfirst.frc.team1257.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends IterativeRobot {

	DriveTrain driveTrain;
	SnailController driveController;
	
	Timer timer;
	
	double lastLeftAccTime = 0;
	double lastLeftJerkTime = 0;
	
	double lastRightAccTime = 0;
	double lastRightJerkTime = 0;
	
	Waypoint[] points;
	Trajectory trajectoryLeft;
	Trajectory trajectoryRight;
	EncoderFollower followerLeft;
	EncoderFollower followerRight;
	
	@Override
	public void robotInit() {
		driveTrain = DriveTrain.getInstance();
		driveController = new SnailController(0);
		
		timer = new Timer();
		timer.start();
		
		EnhancedDashboard.putNumber("Left Pos");
		EnhancedDashboard.putNumber("Left Vel");
		EnhancedDashboard.putNumber("Left Acc");
		EnhancedDashboard.putNumber("Left Jerk");

		EnhancedDashboard.putNumber("Right Pos");
		EnhancedDashboard.putNumber("Right Vel");
		EnhancedDashboard.putNumber("Right Acc");
		EnhancedDashboard.putNumber("Right Jerk");
	}
	
	@Override
	public void autonomousInit() {
		points = new Waypoint[] {
				new Waypoint(-4, -1, Pathfinder.d2r(-45)),
				new Waypoint(-2, -2, 0),
				new Waypoint(0, 0, 0)
		};
		
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 
				0.05, Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION, Constants.MAX_JERK);
		Trajectory trajectory = Pathfinder.generate(points, config);
		
		TankModifier modifier = new TankModifier(trajectory).modify(Constants.WHEEL_BASE_WIDTH_M);
		
		trajectoryLeft = modifier.getLeftTrajectory();
		trajectoryRight = modifier.getRightTrajectory();
		
		followerLeft = new EncoderFollower(trajectoryLeft);
		followerRight = new EncoderFollower(trajectoryRight);
		
		followerLeft.configureEncoder((int) driveTrain.getLeftEncoderPulses(), (int) Constants.PULSES_PER_REV, 
				Constants.inchesToMeters(Constants.WHEEL_DIAMETER));
		followerRight.configureEncoder((int) driveTrain.getRightEncoderPulses(), (int) Constants.PULSES_PER_REV,
				Constants.inchesToMeters(Constants.WHEEL_DIAMETER));
		
		followerLeft.configurePIDVA(Constants.LEFT_P, Constants.LEFT_I, Constants.LEFT_D,
				Constants.LEFT_V, Constants.LEFT_A);
		followerRight.configurePIDVA(Constants.RIGHT_P, Constants.RIGHT_I, Constants.RIGHT_D,
				Constants.RIGHT_V, Constants.RIGHT_A);
	}

	@Override
	public void autonomousPeriodic() {
		double leftOutput = followerLeft.calculate((int) driveTrain.getLeftEncoderPulses());
		double rightOutput = followerRight.calculate((int) driveTrain.getLeftEncoderPulses());
		
		double gyroHeading = driveTrain.getAngle();
		double desiredHeading = Pathfinder.r2d(followerLeft.getHeading());
		
		double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
		
		driveTrain.drive(leftOutput + turn, rightOutput - turn);
	}

	@Override
	public void teleopPeriodic() {
		driveTrain.arcadeDrive(driveController.getForwardSpeed(), driveController.getTurnSpeed());
		
		//Output Information
		double leftVelDiff = driveTrain.getLeftEncoderVelocity() - EnhancedDashboard.getNumber("Left Vel");
		double leftAcc = leftVelDiff / (timer.get() - lastLeftAccTime);
		lastLeftAccTime = timer.get();
		
		double leftAccDiff = leftAcc - EnhancedDashboard.getNumber("Left Acc");
		double leftJerk = leftAccDiff / (timer.get() - lastLeftJerkTime);
		lastLeftJerkTime = timer.get();
		
		EnhancedDashboard.putNumber("Left Pos", driveTrain.getLeftEncoder());
		EnhancedDashboard.putNumber("Left Vel", driveTrain.getLeftEncoderVelocity());
		EnhancedDashboard.putNumber("Left Acc", leftAcc);
		EnhancedDashboard.putNumber("Left Jerk", leftJerk);
		
		double rightVelDiff = driveTrain.getRightEncoderVelocity() - EnhancedDashboard.getNumber("Right Vel");
		double rightAcc = rightVelDiff / (timer.get() - lastRightAccTime);
		lastRightAccTime = timer.get();
		
		double rightAccDiff = rightAcc - EnhancedDashboard.getNumber("Right Acc");
		double rightJerk = rightAccDiff / (timer.get() - lastRightJerkTime);
		lastRightJerkTime = timer.get();

		EnhancedDashboard.putNumber("Right Pos", driveTrain.getRightEncoder());
		EnhancedDashboard.putNumber("Right Vel", driveTrain.getRightEncoderVelocity());
		EnhancedDashboard.putNumber("Right Acc", rightAcc);
		EnhancedDashboard.putNumber("Right Jerk", rightJerk);
	}
}
