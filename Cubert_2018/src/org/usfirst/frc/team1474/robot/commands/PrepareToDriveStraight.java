package org.usfirst.frc.team1474.robot.commands;

import org.usfirst.frc.team1474.robot.Robot;
import org.usfirst.frc.team1474.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1474.robot.subsystems.Gyro2;

import edu.wpi.first.wpilibj.command.Command;

public class PrepareToDriveStraight extends Command{
	public void PrepareToDriveStraight() {
		//RobotDrive.arcadeDrive(0,0);
		requires(Robot.Gyro2);
	}
	// Called just before this Command runs the first time
	// TODO: Pick value for setTimeout(), not sure why 5.
		@Override
		protected void initialize() {
			setTimeout(5);
		}

		// Called repeatedly when this Command is scheduled to run
		@Override
		protected void execute() {
			Gyro2.m_driveStraightHeading = Gyro2.Gyro.getAngle();
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			return false;
		}

		// Called once after isFinished returns true
		// TODO: Figure out what to have the end function be.
		@Override
		protected void end() {
			
		}

		// Called when another command which requires one or more of the same
		// subsystems is scheduled to run
		@Override
		protected void interrupted() {
			end();
		}
}