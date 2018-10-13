package org.usfirst.frc.team1474.robot.commands;

import org.usfirst.frc.team1474.robot.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1474.robot.subsystems.Grabbers;
import org.usfirst.frc.team1474.robot.subsystems.LiftSystem;
import org.usfirst.frc.team1474.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1474.robot.subsystems.Gyro2;
import org.usfirst.frc.team1474.robot.subsystems.Winch;
import org.usfirst.frc.team1474.robot.commands.*; 


public class joystick_control extends Command {
	public joystick_control() {
	requires(Robot.DriveTrain);
	}
	protected void initialize() {
		setTimeout(5);
	}
	protected void execute() {
		DriveTrain.RobotDrive.arcadeDrive(RobotMap.DriverStick.getY(), RobotMap.DriverStick.getX());
		//Robot.DriveTrain.arcadeDrive(RobotMap.DriverStick.getY(), RobotMap.DriverStick.getX());
	}
	protected boolean isFinished() {
		return false;
	}

/**public class IntakeCube extends Command {
	public IntakeCube() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.Grabbers);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		setTimeout(5);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Grabbers.m_Grabbers.set(1.0);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Grabbers.m_Grabbers.set(0.0);
	}
*/
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
