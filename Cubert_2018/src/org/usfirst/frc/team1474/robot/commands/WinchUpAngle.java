/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1474.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1474.robot.Robot;
import org.usfirst.frc.team1474.robot.subsystems.Grabbers;
import org.usfirst.frc.team1474.robot.subsystems.Winch;

public class WinchUpAngle extends Command {
	public WinchUpAngle() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.Winch);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		setTimeout(5);
		//Robot.Winch.WinchUpAngle();
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Winch.WinchUpAngle();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.Winch.Stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
