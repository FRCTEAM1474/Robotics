package org.usfirst.frc.team1474.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1474.robot.Robot;
import org.usfirst.frc.team1474.robot.subsystems.Grabbers;

public class SpitOutCube extends Command {
	public SpitOutCube() {
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
		Grabbers.m_Grabbers.set(-0.78);
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

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
