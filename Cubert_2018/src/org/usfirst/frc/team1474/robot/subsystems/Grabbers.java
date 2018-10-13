package org.usfirst.frc.team1474.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Grabbers extends Subsystem{
	
	static PWMTalonSRX m_frontfrontLeft = new PWMTalonSRX(4);
	static PWMTalonSRX m_frontfrontRight = new PWMTalonSRX(5);
	static SpeedControllerGroup m_Front = new SpeedControllerGroup(m_frontfrontLeft, m_frontfrontRight);
	
	static PWMTalonSRX m_StuffLeft = new PWMTalonSRX(7);
	static PWMTalonSRX m_StuffRight = new PWMTalonSRX(8);
	static SpeedControllerGroup m_Stuff = new SpeedControllerGroup(m_StuffLeft, m_StuffRight);
	
	public static SpeedControllerGroup m_Grabbers = new SpeedControllerGroup(m_Front, m_Stuff);
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}	
	
public void IntakeCube() {
}

public void IntakeGoToBackAt20Percent() {
}

public static void SpitOutCube() {	
}


}