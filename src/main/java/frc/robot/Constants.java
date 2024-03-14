/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static Translation2d powerPortLocation = new Translation2d(15.98, 2.43);

    //Drivetrain
    public static double drivetrainAcceleration = 0.25; //Seconds from neutral to full
    public static double drivetrainMaxSpeed = 1;
    public static double relativePulsesPerRevolution = 1662.25;

    //Xbox
    public static int visionAlignButtonNumber = 6;
    public static int resetGyroButtonNumber = 5;

    //Joystick
    public static int intakeForwardsButtonNumber = 3;
    public static int intakeBackwardsButtonNumber = 4;
    public static int spinUpFlywheelButtonNumber = 2;
    public static int shootButtonNumber = 1;
    public static int climbButtonNumber = 7;
    public static int reverseClimberButtonNumber = 6;

    public static double[] joystickSensitivityRange = {0.35, drivetrainMaxSpeed};
    public static double joystickDeadZone = 0.1;

    //CAN
    public static int frontLeftDriveMotorNumber = 1;
    public static int frontRightDriveMotorNumber = 2;
    public static int backLeftDriveMotorNumber = 3;
    public static int backRightDriveMotorNumber = 4;
    public static int frontLeftSteeringMotorNumber = 5;
    public static int frontRightSteeringMotorNumber = 6;
    public static int backLeftSteeringMotorNumber = 7;
    public static int backRightSteeringMotorNumber = 8;
    public static int shooterMotorNumber = 9;

    //PWM
    public static int gateSparkPort = 0;

    //Relay
    public static int visionLightsRelayPort = 0;
    public static int frontIntakeRelayPort = 1;
    public static int backIntakeRelayPort = 2;
    public static int winchRelayPort = 3;

    //Analog In
    public static int ultrasonicPort = 1;
}
