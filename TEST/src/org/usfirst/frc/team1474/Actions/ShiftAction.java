/**package org.usfirst.frc.team1474.Actions;

import org.usfirst.frc.team1474.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team1474.Utilities.CustomAction;

public class ShiftAction extends CustomAction {
	private DriveBaseSubsystem driveBaseSubsystem;
	
	public ShiftAction() {
		super();
		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
	}

	@Override
	public void start() {
		;
	}
	
	public void start(boolean highGear) {
		driveBaseSubsystem.setGear(highGear);
	}
	
	@Override
	public void run() {
		;
	}

}**/