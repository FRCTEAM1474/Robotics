/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1474.robot;

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
import org.usfirst.frc.team1474.robot.OI;
import org.usfirst.frc.team1474.robot.RobotMap;
import org.usfirst.frc.team1474.robot.commands.*; 

public class Robot extends IterativeRobot 
{
	public static final org.usfirst.frc.team1474.robot.subsystems.Grabbers Grabbers = new Grabbers();
	public static final org.usfirst.frc.team1474.robot.subsystems.LiftSystem LiftSystem = new LiftSystem();
	public static OI m_oi;
	public static final org.usfirst.frc.team1474.robot.subsystems.DriveTrain DriveTrain = new DriveTrain();
	public static final org.usfirst.frc.team1474.robot.subsystems.Gyro2 Gyro2 = new Gyro2();
	public static final org.usfirst.frc.team1474.robot.subsystems.Winch Winch = new Winch();
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		m_oi = new OI();
		SmartDashboard.putData("Auto mode", m_chooser);
	
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit()
	{

	}

	@Override
	public void disabledPeriodic() 
	{
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() 
	{
		m_chooser.addDefault("DriveStraight", new DriveStraighht(-.6));
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) 
		{
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() 
	{
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) 
		{
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() 
	{
		Scheduler.getInstance().run();
	}
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() 
	{
	}
}
