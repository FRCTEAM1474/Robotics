package org.usfirst.frc.team1474.Utilities;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.SPI;
import org.usfirst.frc.team1474.Utilities.Drivers.CANSpeedControllerBuilder;
import org.usfirst.frc.team1474.Utilities.Drivers.CKTalonSRX;
import org.usfirst.frc.team1474.Utilities.Drivers.KnightJoystick;
import org.usfirst.frc.team1474.Utilities.Drivers.NavX;

public class Controllers {              //All controllers are declared in this class, which have function of controlling something on the bot

    private static Controllers instance = null;             //singleton (reference 1st comment in DriveBaseSystem to know what singleton is)

    public static Controllers getInstance() {
        if (instance == null) {
            instance = new Controllers();
        }
            return instance;
    }

    private Controllers() {

        //Choose whether to create Victor or Talon slaves here
        //Left2Cube Drive Setup
        leftDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(Constants.kLeftDriveMasterId, Constants.kLeftDriveMasterPDPChannel);                    //references CanSpeedControllerBuilder.java
        leftDrive2 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId, Constants.kLeftDriveSlave1PDPChannel, leftDrive1);
        leftDrive3 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId2, Constants.kLeftDriveSlave2PDPChannel, leftDrive1);

        //Right2Cube Drive Setup
        rightDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(Constants.kRightDriveMasterId, Constants.kRightDriveMasterPDPChannel);
        rightDrive2 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId, Constants.kRightDriveSlave1PDPChannel, rightDrive1);
        rightDrive3 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId2, Constants.kRightDriveSlave2PDPChannel, rightDrive1);

        try {
            navX = new NavX(SPI.Port.kMXP);
        } catch (Exception ex) {
        }

        canifierLED = new CANifier(Constants.kCANifierLEDId);

        driveJoystickThrottle = new KnightJoystick(0);

    }




    private CKTalonSRX leftDrive1;
    private BaseMotorController leftDrive2;         //BaseMotorControllers are the slave controllers (the ones connected off the first one by CAN)
    private BaseMotorController leftDrive3;
    private CKTalonSRX rightDrive1;                 //CKTalonSRX is a driver / wrapper made by 195 for Talon SRX
    private BaseMotorController rightDrive2;
    private BaseMotorController rightDrive3;

    private KnightJoystick driveJoystickThrottle;

    private NavX navX;

    private CANifier canifierLED;

    public CKTalonSRX getLeftDrive1() {
        return leftDrive1;
    }

    public CKTalonSRX getRightDrive1() {
        return rightDrive1;
    }

    public BaseMotorController getLeftDrive2() {
        return leftDrive2;
    }

    public BaseMotorController getRightDrive2() {
        return rightDrive2;
    }

    public BaseMotorController getLeftDrive3() {
        return leftDrive3;
    }

    public BaseMotorController getRightDrive3() {
        return rightDrive3;
    }

    public NavX getNavX() {
        return navX;
    }

    public CANifier getCANifierLED() {
        return canifierLED;
    }

    public KnightJoystick getDriveJoystickThrottle() {
        return driveJoystickThrottle;
    }

}





