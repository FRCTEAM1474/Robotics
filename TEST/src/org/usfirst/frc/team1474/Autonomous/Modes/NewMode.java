package org.usfirst.frc.team1474.Autonomous.Modes;


import org.usfirst.frc.team1474.Actions.DrivePathAction;
import org.usfirst.frc.team1474.Actions.FlashLEDsAction;
import org.usfirst.frc.team1474.Actions.Framework.ParallelAction;
import org.usfirst.frc.team1474.Actions.Framework.SeriesAction;
import org.usfirst.frc.team1474.Actions.Framework.WaitForPathMarkerAction;
import org.usfirst.frc.team1474.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1474.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team1474.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team1474.Autonomous.Paths.MarkerPath;
import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class NewMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer pathContainer = new MarkerPath();
        runAction(new ResetPoseFromPathAction(pathContainer));

        runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),          //Robot Drives the set path, but at the marker point, the "intake" would come down
                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("OurMarker"),
                        new FlashLEDsAction())))));
    }
}
