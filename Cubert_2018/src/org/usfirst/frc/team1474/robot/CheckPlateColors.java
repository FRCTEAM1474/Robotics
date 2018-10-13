
package org.usfirst.frc.team1474.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class CheckPlateColors {

	private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
	private static NetworkTable table = tableInstance.getTable("PyDashboard");

	public static boolean isLeftSwitchCorrectColor() {
		return messageToCharArray()[0] == 'L';
	}
	
	public static boolean isRightSwitchCorrectColor() {
		return messageToCharArray()[0] == 'R';
	}
	
	public static boolean isLeftScaleCorrectColor() {
		return messageToCharArray()[1] == 'L';
	}

	public static boolean isRightScaleCorrectColor() {
		return messageToCharArray()[1] == 'R';
	}

	public static char[] messageToCharArray() {
		return DriverStation.getInstance().getGameSpecificMessage().toCharArray(); 
	}

	public static boolean isSwitchCorrectColor() {
		if(getStartingPosition().equals("Left")) {
			return isLeftSwitchCorrectColor();
		} else {
			return isRightSwitchCorrectColor();
		}
	}

	public static boolean isScaleCorrectColor() {
		if(getStartingPosition().equals("Left")) {
			return isLeftScaleCorrectColor();
		} else {
			return isRightScaleCorrectColor();
		}
	}

	public static String getStartingPosition() {
		return table.getEntry("startingPosition").getString("unknown");
	}
	
	public static boolean useDefault() {
		String startingPosition = getStartingPosition();
		System.out.println("Starting position: " + startingPosition);
		return !startingPosition.equals("Left") && !startingPosition.equals("Center") && !startingPosition.equals("Right"); 
	}

	public static double getTimeToWait() {
		return Double.parseDouble(table.getEntry("timetowait").getString("0"));
	}

}