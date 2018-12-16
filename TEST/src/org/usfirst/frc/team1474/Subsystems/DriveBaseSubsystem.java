package org.usfirst.frc.team1474.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import org.usfirst.frc.team1474.Utilities.*;
import org.usfirst.frc.team1474.Utilities.Drivers.CKTalonSRX;
import org.usfirst.frc.team1474.Utilities.Drivers.NavX;
import org.usfirst.frc.team1474.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team1474.Utilities.Loops.Loop;
import org.usfirst.frc.team1474.Utilities.Loops.Looper;
import org.usfirst.frc.team1474.Utilities.TrajectoryFollowingMotion.*;

import java.util.concurrent.locks.ReentrantLock;


public class DriveBaseSubsystem implements CustomSubsystem {
    private static DriveBaseSubsystem instance = null;          //DriveBaseSubsystem considered singleton; only be (1) instance of the class;
    private CKTalonSRX mLeftMaster, mRightMaster;
    private BaseMotorController leftDriveSlave1, leftDriveSlave2, rightDriveSlave1, rightDriveSlave2;
    private NavX mNavXBoard;
    private DriveControlState mControlMode;
    private static ReentrantLock _subsystemMutex = new ReentrantLock();
    private boolean mPrevBrakeModeVal;
    private Path mCurrentPath = null;
    private PathFollower mPathFollower;
    private PathFollowerRobotState mRobotState = PathFollowerRobotState.getInstance();

    public static DriveBaseSubsystem getInstance() {            //Any time use class, return static instance; can use instance data all the time, but only established once
        if (instance == null)
            instance = new DriveBaseSubsystem();

        return instance;
    }

    private DriveBaseSubsystem() {          //Constructor private so no other class called DriveBaseSubsystem

        Controllers robotControllers = Controllers.getInstance();
        mLeftMaster = robotControllers.getLeftDrive1();
        leftDriveSlave1 = robotControllers.getLeftDrive2();
        leftDriveSlave2 = robotControllers.getLeftDrive3();
        mRightMaster = robotControllers.getRightDrive1();
        rightDriveSlave1 = robotControllers.getRightDrive2();
        rightDriveSlave2 = robotControllers.getRightDrive3();

        mNavXBoard = robotControllers.getNavX();

        mPrevBrakeModeVal = false;          //instantiate brake mode so there is stored value and not relying on loop

        mControlMode = DriveControlState.PATH_FOLLOWING;        //initialize control to path following, since that is what it will be doing at start
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onFirstStart(double timestamp) {            //very first time loop is run
            synchronized (DriveBaseSubsystem.this) {
                subsystemHome();                //ensures gyro and encoders are set to 0
            }
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (DriveBaseSubsystem.this) {
                setDriveOpenLoop(DriveMotorValues.NEUTRAL);         //runs every time looper is started
                setBrakeMode(false);
                setDriveVelocity(new DriveMotorValues(0, 0));       //requests a velocity output from each talon on each side of the drivetrain
            }
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {          //Will run as the loop
            synchronized (DriveBaseSubsystem.this) {
//				SmartDashboard.putNumber("Left Drive Velocity", getLeftVelocityInchesPerSec());
//				SmartDashboard.putNumber("Right Drive Velocity", getRightVelocityInchesPerSec());
                switch (mControlMode) {
                    case OPEN_LOOP:
                        break;
                    case VELOCITY:
                        break;
                    case TURN_TO_HEADING:
                        break;
                    case PATH_FOLLOWING:
                        if (mPathFollower != null) {
                            updatePathFollower(timestamp);
                            //mCSVWriter.add(mPathFollower.getDebug());
                        }
                        break;
                    default:
                        break;
                }

            }
        }
        @Override
        public void onStop(double timestamp) {              //Run when the loop is stopped
            setDriveOpenLoop(DriveMotorValues.NEUTRAL);
        }
    };


    @Override
    public void init() {                //Inside init, do actual .set
        mLeftMaster.setSensorPhase(true);

        mRightMaster.setSensorPhase(true);
        mRightMaster.setInverted(true);
        rightDriveSlave1.setInverted(true);
        rightDriveSlave2.setInverted(true);

        setBrakeMode(true);

        boolean setSucceeded;
        int retryCounter = 0;

        do {
            setSucceeded = true;

            setSucceeded &= mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
            setSucceeded &= mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
            setSucceeded &= mLeftMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

            setSucceeded &= mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
            setSucceeded &= mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
            setSucceeded &= mRightMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

        } while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

        //setSucceeded &= TalonHelper.setPIDGains(mLeftMaster, 0, Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf, Constants.kDriveLowGearPositionRampRate, Constants.kDriveLowGearPositionIZone);         //declare this method for shifting
        setSucceeded &= TalonHelper.setPIDGains(mLeftMaster, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
        //setSucceeded &= TalonHelper.setPIDGains(mRightMaster, 1, Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf, Constants.kDriveLowGearPositionRampRate, Constants.kDriveLowGearPositionIZone);        //declare this method for shifting
        setSucceeded &= TalonHelper.setPIDGains(mRightMaster, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
        setSucceeded &= TalonHelper.setMotionMagicParams(mLeftMaster, (int)Constants.kDriveLowGearMaxVelocity, (int)Constants.kDriveLowGearMaxAccel);
        setSucceeded &= TalonHelper.setMotionMagicParams(mRightMaster, (int)Constants.kDriveLowGearMaxVelocity, (int)Constants.kDriveLowGearMaxAccel);

        mLeftMaster.selectProfileSlot(0, 0);
        mRightMaster.selectProfileSlot(0, 0);
    }

    @Override                   //can't override subsystemHome if there is no base class to override
    public void subsystemHome() {
        mNavXBoard.zeroYaw();                   //NavX has gyro on board (NEEDS TO BE CHANGED WITH DIFFERENT GYRO)

        boolean setSucceeded;
        int retryCounter = 0;

        do {
            setSucceeded = true;

            setSucceeded &= mLeftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
            setSucceeded &= mRightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
        } while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

        //if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
           // ConsoleReporter.report("Failed to zero DriveBaseSubsystem!!!", MessageLevel.DEFCON1);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public void setControlMode(DriveControlState controlMode) {             //sends differential updates to the control mode; if control mode not equal control mode, then change it
        if (controlMode != mControlMode) {
            try {
                _subsystemMutex.lock();         //Mutex is a way of synchronizing things across threads; make sure to unlock mutex
                mControlMode = controlMode;
            } catch (Exception ex) {

            } finally {
                _subsystemMutex.unlock();
            }
        }
    }

    public void setBrakeMode(boolean brakeMode) {           //controls all talons at once; only does update if request is different than before
        if (mPrevBrakeModeVal != brakeMode) {
            _subsystemMutex.lock();
            mLeftMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            leftDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            leftDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            mRightMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            rightDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            rightDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            mPrevBrakeModeVal = brakeMode;
            _subsystemMutex.unlock();
        }
    }

    public synchronized void setDriveOpenLoop (DriveMotorValues d) {            //DriveMotorValues holds values for motors so only 1 variable is passed at same time
        setControlMode(DriveControlState.OPEN_LOOP);

        mLeftMaster.set(ControlMode.PercentOutput, d.leftDrive);            //ControlMode tells the talon what mode it should be running (Phoenix Library)
        mRightMaster.set(ControlMode.PercentOutput, d.rightDrive);
    }

    public synchronized void setDriveVelocity(DriveMotorValues d) {         //2 methods because times where setDriveVelocity needed without mode change (PATH_FOLLOWING is example)
        setDriveVelocity(d, true);
    }

    public synchronized void setDriveVelocity(DriveMotorValues d, boolean autoChangeMode) {
        if (autoChangeMode)
            setControlMode(DriveControlState.VELOCITY);
        mLeftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.leftDrive));     //Need to convert RPM (Revolutions Per Minute) to Native Units
        mRightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.rightDrive));
    }

    private void updatePathFollower(double timestamp) {                 //Method for path following
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();             //mRobotState another class that can be imported
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                PathFollowerRobotState.getInstance().getDistanceDriven(), PathFollowerRobotState.getInstance().getPredictedVelocity().dx);

        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updatePathVelocitySetpoint(setpoint.left, setpoint.right);


        } else {
            updatePathVelocitySetpoint(0, 0);
            setControlMode(DriveControlState.VELOCITY);
        }
    }

    private void updatePathVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
        final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;

        mLeftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale)));
        mRightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale)));

    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) + 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }


    public double getLeftDistanceInches() {
        return rotationsToInches(mLeftMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
    }

    public double getRightDistanceInches() {
        return rotationsToInches(mRightMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0)));
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0)));
    }

    public synchronized Rotation2d getGyroAngle() {
        return mNavXBoard.getYaw();
    }

    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle);
    }

    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mControlMode != DriveControlState.PATH_FOLLOWING) {
            setControlMode(DriveControlState.PATH_FOLLOWING);
            PathFollowerRobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));

            mCurrentPath = path;
        } else {

        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            if (mPathFollower != null)
                return mPathFollower.isFinished();
            else
                return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            if (mPathFollower != null)
                return (mPathFollower.isFinished() || mPathFollower.hasPassedMarker(marker));
            else {
                //TODO: Test with false value
                return true;
            }
        }
    }
}
