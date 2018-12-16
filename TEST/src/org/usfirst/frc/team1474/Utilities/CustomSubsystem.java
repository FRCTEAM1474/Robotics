package org.usfirst.frc.team1474.Utilities;

import org.usfirst.frc.team1474.Utilities.Loops.Looper;

public interface CustomSubsystem {
	void init();
	void subsystemHome();
	void registerEnabledLoops(Looper in);
}
