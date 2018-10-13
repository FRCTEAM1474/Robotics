package org.usfirst.frc.team1474.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1474.robot.Robot;
import org.usfirst.frc.team1474.robot.subsystems.DriveTrain;

public class EndTimer extends Command{
		public EndTimer() {
			// Use requires() here to declare subsystem dependencies
			requires(Robot.DriveTrain);
		}

		// Called just before this Command runs the first time
		@Override
		protected void initialize() {
			DriveTrain.m_timer.start();
			
		}
		
		// Called repeatedly when this Command is scheduled to run
		@Override
		protected void execute() {
			DriveTrain.m_timer.reset();
		}

		// Make this return true when this Command no longer needs to run execute()
		
		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
				return false;
			}

			// Called once after isFinished returns true
			@Override
			protected void end() {
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
