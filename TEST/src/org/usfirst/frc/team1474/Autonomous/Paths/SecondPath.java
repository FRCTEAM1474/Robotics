package org.usfirst.frc.team1474.Autonomous.Paths;

import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class SecondPath implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(120,275,0,0));
        sWaypoints.add(new Waypoint(245,302,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 276), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
