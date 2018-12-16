package org.usfirst.frc.team1474.Actions;

import org.usfirst.frc.team1474.Actions.Framework.Action;
import org.usfirst.frc.team1474.LEDController;

public class FlashLEDsAction implements Action {

    public FlashLEDsAction () {

    }

    @Override
    public boolean isFinished() {
        return true;
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
        LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
    }
}
