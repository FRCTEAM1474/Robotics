package org.usfirst.frc.team1474.robot.commands;

import org.usfirst.frc.team1474.robot.Robot;
import org.usfirst.frc.team1474.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1474.robot.subsystems.Grabbers;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team1474.robot.RobotMap;

public class Stop extends Command {
	
	public void Stop() {
		//RobotDrive.arcadeDrive(0,0);
		requires(Robot.DriveTrain);
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
			DriveTrain.RobotDrive.arcadeDrive(0,0);
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			return false;
		}

		// Called once after isFinished returns true
		// TODO: Figure out what to have the end function be. Currently set it as stopped (arcadeDrive(0,0)), just because I can't imagine what else to do
		@Override
		protected void end() {
			DriveTrain.RobotDrive.arcadeDrive(0,0);
		}

		// Called when another command which requires one or more of the same
		// subsystems is scheduled to run
		@Override
		protected void interrupted() {
			end();
		}
}
