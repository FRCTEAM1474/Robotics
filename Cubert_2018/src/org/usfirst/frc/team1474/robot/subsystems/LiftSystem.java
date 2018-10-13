package org.usfirst.frc.team1474.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class LiftSystem extends Subsystem {

	public static PWMTalonSRX m_TalonLift = new PWMTalonSRX(6);
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	} 
	public void LiftGoUp() {
	}

	public void LiftGoDown() {
	}
	
	public void LiftGoUpSlow() {
	}
	
    public void LiftGoDownSlow() {
    }
    
	}
