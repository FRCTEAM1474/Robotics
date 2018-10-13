/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1474.robot;

import org.usfirst.frc.team1474.robot.RobotMap;
import org.usfirst.frc.team1474.robot.commands.IntakeCube;
import org.usfirst.frc.team1474.robot.commands.IntakeGoToBackAt20Percent;
import org.usfirst.frc.team1474.robot.commands.LiftGoDown;
import org.usfirst.frc.team1474.robot.commands.LiftGoDownSlow;
import org.usfirst.frc.team1474.robot.commands.LiftGoUp;
import org.usfirst.frc.team1474.robot.commands.LiftGoUpSlow;
import org.usfirst.frc.team1474.robot.commands.SpitOutCube;
import org.usfirst.frc.team1474.robot.commands.WinchDownAngle;
import org.usfirst.frc.team1474.robot.commands.WinchUpAngle;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI 
{

public OI() 
{
	RobotMap.IntakeCubeButton.whileHeld(new IntakeCube());

	RobotMap.IntakeGoToBackAt20PercentButton.whileHeld(new IntakeGoToBackAt20Percent());

	RobotMap.SpitOutCube.whileHeld(new SpitOutCube());

	RobotMap.LiftGoUpButton.whileHeld(new LiftGoUp());

	RobotMap.LiftGoUpSlowButton.whileHeld(new LiftGoUpSlow());

	RobotMap.LiftGoDownButton.whileHeld(new LiftGoDown());
	
	RobotMap.LiftGoDownSlowButton.whileHeld(new LiftGoDownSlow());

	RobotMap.WinchUpButton.whileHeld(new WinchUpAngle());

	RobotMap.WinchDownButton.whileHeld(new WinchDownAngle());

}

}