package org.usfirst.frc.team1474.robot.subsystems;

import org.usfirst.frc.team1474.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro2 extends Subsystem {

public static ADXRS450_Gyro Gyro = new ADXRS450_Gyro();

static double m_driveStraightHeading; 

		@Override
		protected void initDefaultCommand() {
			// TODO Auto-generated method stub
			
		}

		
public void prepareToDriveStraight() {
	m_driveStraightHeading = Gyro.getAngle();
}
		
public static void driveStraight(double speed) {
	double currentHeading = Gyro.getAngle();
	double currentOffsetFromDesiredHeading = currentHeading - m_driveStraightHeading; 
	double rotationSpeed = currentOffsetFromDesiredHeading * 0.19;
	
	DriveTrain.RobotDrive.arcadeDrive(speed, -rotationSpeed);
}

public static void Turn(double angle) {
	double currentHeading = Gyro.getAngle();
	double currentOffsetFromDesiredHeading = angle- currentHeading; 
	double rotationSpeed = currentOffsetFromDesiredHeading * 0.24;
	
	DriveTrain.RobotDrive.arcadeDrive(rotationSpeed,-rotationSpeed);

}

public void Stop() {
	DriveTrain.RobotDrive.arcadeDrive(0,0);
}
}



