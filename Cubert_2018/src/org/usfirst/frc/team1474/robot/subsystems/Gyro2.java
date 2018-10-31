package org.usfirst.frc.team1474.robot.subsystems;

import org.usfirst.frc.team1474.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro2 extends Subsystem {

public static ADXRS450_Gyro Gyro = new ADXRS450_Gyro();

public static double m_driveStraightHeading; 

		@Override
		protected void initDefaultCommand() {
			// TODO Auto-generated method stub
			
		}

		
public void PrepareToDriveStraight() {//moved to Commands
	//m_driveStraightHeading = Gyro.getAngle();
}
		
public static void driveStraight(double speed) {//TODO: finish moving to Commands.
	double currentHeading = Gyro.getAngle();
	double currentOffsetFromDesiredHeading = currentHeading - m_driveStraightHeading; 
	double rotationSpeed = currentOffsetFromDesiredHeading * 0.19;
	
	DriveTrain.RobotDrive.arcadeDrive(speed, -rotationSpeed);
}

public static void Turn(double angle) {//TODO: Move to commands
	double currentHeading = Gyro.getAngle();
	double currentOffsetFromDesiredHeading = angle- currentHeading; 
	double rotationSpeed = currentOffsetFromDesiredHeading * 0.24;
	
	DriveTrain.RobotDrive.arcadeDrive(rotationSpeed,-rotationSpeed);

}

public void Stop() {//TODO: Move to commands/figure out whether I need to because its duplicate of the drivetrain stop() which is already in commands.
	DriveTrain.RobotDrive.arcadeDrive(0,0);
}
}



