package org.usfirst.frc.team1474.Autonomous.Modes;


import org.usfirst.frc.team1474.Actions.DrivePathAction;
import org.usfirst.frc.team1474.Actions.Framework.WaitAction;
import org.usfirst.frc.team1474.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1474.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team1474.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team1474.Autonomous.Paths.FirstPath;
import org.usfirst.frc.team1474.Autonomous.Paths.SecondPath;
import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.PathContainer;

public class BasicMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new FirstPath();				//line to declare which auto mode wanting execution
		runAction(new ResetPoseFromPathAction(pathContainer));					//resets estimated position to expected starting position

		runAction(new DrivePathAction(pathContainer));

		runAction(new WaitAction(2));

		runAction(new DrivePathAction(new SecondPath()));

		runAction(new WaitAction(15));
	}
}
