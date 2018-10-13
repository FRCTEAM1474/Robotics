package org.usfirst.frc.team1474.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Winch extends Subsystem{
	
	public static Spark m_Winch = new Spark(9);

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}	
public static void WinchUpAngle() {
	m_Winch.set(1.0);
}

public void WinchDownAngle() {
	m_Winch.set(-0.50);
}
public void Stop() {
	m_Winch.set(0.0);
}
	
}

