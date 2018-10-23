package org.usfirst.frc.team1474.robot.commands;

import org.usfirst.frc.team1474.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1474.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1474.robot.subsystems.Gyro2;

public class DriveStraighht extends Command 
{
	public double speed;

	public DriveStraighht(double y) {
		speed = y;
		// Use requires() here to declare subsystem dependencies
		requires(Robot.Gyro2);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		DriveTrain.m_timer.start();
		Robot.Gyro2.prepareToDriveStraight();
      
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (DriveTrain.m_timer.get() < 4.1) {	
		Gyro2.driveStraight(-0.6);
		// 4.1 seconds of autonomous driving? backwards at 60%
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.Gyro2.Stop();
		DriveTrain.m_timer.reset();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		DriveTrain.m_timer.reset();
		end();
}
}


