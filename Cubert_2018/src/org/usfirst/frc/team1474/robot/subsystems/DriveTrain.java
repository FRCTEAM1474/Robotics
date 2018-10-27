package org.usfirst.frc.team1474.robot.subsystems;

import org.usfirst.frc.team1474.robot.RobotMap;
import org.usfirst.frc.team1474.robot.commands.joystick_control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrain extends Subsystem{
	
	static PWMTalonSRX m_frontLeft = new PWMTalonSRX(0);
	static PWMTalonSRX m_rearLeft = new PWMTalonSRX(2);
	static SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);


	static PWMTalonSRX m_frontRight = new PWMTalonSRX(1);
	static PWMTalonSRX m_rearRight = new PWMTalonSRX(3);
	static SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);


	public static DifferentialDrive RobotDrive = new DifferentialDrive(m_left, m_right);
//public static DifferentialDrive RobotDrive;	
	
	public static Timer m_timer = new Timer();
	
	public void arcadeDrive(double x, double y) {
		RobotDrive.arcadeDrive(x, y);
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new joystick_control());
		// TODO Auto-generated method stub
		
	}
	public void Stop() {
		
	}
/* Put Stop() as own command in Commands folder. -Casey
	public void Stop() {
		RobotDrive.arcadeDrive(0,0);
	}
*/	
}
