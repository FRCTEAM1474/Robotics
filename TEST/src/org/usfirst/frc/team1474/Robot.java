/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1474;


import org.usfirst.frc.team1474.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team1474.Autonomous.Framework.AutoModeExecuter;
import org.usfirst.frc.team1474.Autonomous.Modes.NewMode;
import org.usfirst.frc.team1474.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team1474.Utilities.*;
import org.usfirst.frc.team1474.Utilities.Drivers.KnightJoystick;
import org.usfirst.frc.team1474.Utilities.Loops.Looper;
import org.usfirst.frc.team1474.Utilities.Loops.RobotStateEstimator;
import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.Util;

public class Robot extends testBot {

    private Controllers robotControllers;
    //private ArrayList<CustomSubsystem> subsystemVector;             //ArrayList is list of every subsystem wanted on the robot; needed with more than one

    private DriveBaseSubsystem driveBaseSubsystem;
    private RobotStateEstimator robotStateEstimator;
    private Looper mLooper;
    private ThreadRateControl threadRateControl = new ThreadRateControl();
    private AutoModeExecuter autoModeExecuter;
    private LEDController ledController;
    private KnightJoystick driveJoystickThrottle;



    @Override
    public void robotInit()             //Methods robot will run once it reaches each state; executed once only
    {
        //threadRateControl.start(true);                            //Needed with more than one subsystem

        //for (CustomSubsystem customSubsystem : subsystemVector) {                 //Needed with more than one subsystem
           // customSubsystem.init();                   //Needed with more than one subsystem
            //threadRateControl.doRateControl(100);         //Needed with more than one subsystem
        //}

        //for (CustomSubsystem customSubsystem : subsystemVector) {         //Needed with more than one subsystem
          //  customSubsystem.registerEnabledLoops(mLooper);            //Needed with more than one subsystem
        //}

        robotControllers = Controllers.getInstance();
        mLooper = new Looper();

        driveBaseSubsystem = driveBaseSubsystem.getInstance();
        driveBaseSubsystem.init();
        driveBaseSubsystem.registerEnabledLoops(mLooper);           //Register drivetrain as a loop
        robotStateEstimator = RobotStateEstimator.getInstance();
        mLooper.register(robotStateEstimator);
        ledController = LEDController.getInstance();
        ledController.start();
        ledController.setRequestedState(LEDController.LEDState.FIXED_ON);

        driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();

    }

    @Override
    public void autonomous() {

        mLooper.start(true);
        driveBaseSubsystem.setBrakeMode(true);
        autoModeExecuter = new AutoModeExecuter();

        AutoModeBase autoMode = new NewMode();

        if (autoMode != null)
            autoModeExecuter.setAutoMode(autoMode);
        else
            return;

        autoModeExecuter.start();

        threadRateControl.start(true);

        while (isAutonomous() && isEnabled()) {threadRateControl.doRateControl(100);}
    }

    @Override
    public void operatorControl()
    {
        exitAuto();
        mLooper.start(false);

        double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband);
        double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband);

        driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(y + x, 1), Util.limit(y-x, 1)));
        while (isOperatorControl() && isEnabled()) {
            threadRateControl.doRateControl(100);
        }
    }

    @Override
    public void disabled()
    {
        exitAuto();                   //call exitAuto b/c auto ends, then robot is stuck in path following mode

        mLooper.stop();

        threadRateControl.start(true);          //wrapper so that you don't have to do try catch every time

        while (isDisabled()) {
            driveBaseSubsystem.setBrakeMode(false);
            threadRateControl.doRateControl(100);
        }

    }

    private void exitAuto() {
        try {
            if (autoModeExecuter != null)           //autoModeExecuter is thing that actually runs autonomous mode
                autoModeExecuter.stop();

            autoModeExecuter = null;
        } catch (Throwable t){

        }
    }

    @Override
    public void test()
    {

    }
}
