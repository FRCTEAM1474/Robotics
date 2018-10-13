package org.usfirst.frc.team1474.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

public static Joystick DriverStick = new Joystick(0);
static Joystick NonDriverStick = new Joystick(1);

static Button IntakeCubeButton = new JoystickButton(NonDriverStick, 1);
static Button IntakeGoToBackAt20PercentButton = new JoystickButton(DriverStick, 2);
static Button SpitOutCube = new JoystickButton(DriverStick, 3);

static Button LiftGoUpButton = new JoystickButton(NonDriverStick, 4);
static Button LiftGoUpSlowButton = new JoystickButton(NonDriverStick, 5);
static Button LiftGoDownButton = new JoystickButton(NonDriverStick, 6);
static Button LiftGoDownSlowButton = new JoystickButton(NonDriverStick, 7);

static Button WinchUpButton = new JoystickButton(DriverStick, 6);
static Button WinchDownButton = new JoystickButton(DriverStick, 4);

//static Button ThirdButtonDriverStick = new JoystickButton(DriverStick, 3);

//static Button FourthButtonDriverStick = new JoystickButton(DriverStick, 4);

//static Button FifthButtonDriverStick = new JoystickButton(DriverStick, 5);

//static Button SixthButtonDriverStick = new JoystickButton(DriverStick, 6);

//static Button SeventhButtonDriverStick = new JoystickButton(DriverStick, 7);

//static Button EighthButtonDriverStick = new JoystickButton(DriverStick, 8);

//static Button NinthButtonDriverStick = new JoystickButton(DriverStick, 9);

//static Button TenthButtonDriverStick = new JoystickButton(DriverStick, 10);

//static Button EleventhButtonDriverStick = new JoystickButton(DriverStick, 11);

//static Button TwelthButtonDriverStick = new JoystickButton(DriverStick, 12);
}
