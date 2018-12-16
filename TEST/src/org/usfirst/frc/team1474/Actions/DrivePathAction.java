package org.usfirst.frc.team1474.Actions;

import org.usfirst.frc.team1474.Actions.Framework.Action;
import org.usfirst.frc.team1474.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.Path;
import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.PathContainer;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private DriveBaseSubsystem mDrive = DriveBaseSubsystem.getInstance();

    public DrivePathAction(PathContainer p) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }
}
