package org.usfirst.frc.team1474.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team1474.robot.Robot;
import org.usfirst.frc.team1474.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1474.robot.subsystems.Gyro2;
import org.usfirst.frc.team1474.robot.commands.Stop;

public class DriveStraight extends Command {
	
	public double speed;
	
	public DriveStraight(double y) {
		speed = y;
		// Use requires() here to declare subsystem dependencies
		requires(Robot.Gyro2);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		DriveTrain.m_timer.start();
		//Robot.Gyro2.prepareToDriveStraight();
		//Operator must manually select PrepareToDriveStraight, after command was moved from subsystem to commands.
      
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (DriveTrain.m_timer.get() < 4.1) {	
			double currentHeading = Robot.Gyro2.Gyro.getAngle();
			double currentOffsetFromDesiredHeading = currentHeading - Gyro2.m_driveStraightHeading; 
			double rotationSpeed = currentOffsetFromDesiredHeading * 0.19;
			
			DriveTrain.RobotDrive.arcadeDrive(speed, -rotationSpeed);
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
		Robot.Gyro2.Stop();//TODO: Change this to command stop, not the one in Gyro2.
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


