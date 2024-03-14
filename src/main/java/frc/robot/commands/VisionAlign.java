/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLights;

import java.util.function.DoubleSupplier;

/**
 * Rotate the robot towards the goal using the vision system
 */
public class VisionAlign extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Drivetrain drivetrain;
    private final VisionLights visionSubsystem;
    private final NetworkTableEntry getAngleToTarget;
    private final NetworkTableEntry getTargetFound;

    private boolean endWhenAligned;

    private double targetAngle;

    private NetworkTable visionTable;

    private double lastAngleToTarget = 1000; //Make sure to start on to always be different from the first reading

    /**
     * Consturctor for the VisionAlign command
     *
     * @param drivetrainSubsystem The Drivetrain subsystem
     * @param visionLightsSubsystem The VisionLights subsystem
     * @param endWhenAligned Whether the command should exit when it detects it's aligned
     */
    public VisionAlign(Drivetrain drivetrainSubsystem, VisionLights visionLightsSubsystem, boolean endWhenAligned) {
        this.endWhenAligned = endWhenAligned;

        drivetrain = drivetrainSubsystem;
        visionSubsystem = visionLightsSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        addRequirements(visionSubsystem);

        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("vision");

        getAngleToTarget = visionTable.getEntry("AngleToTarget");
        getTargetFound = visionTable.getEntry("TargetFound");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        visionSubsystem.setLightState(true);
        drivetrain.configurePIDLoop(-0.05, 0.001, 0);
        drivetrain.initRotationPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("VisionAlign");

        if (getTargetFound.getBoolean(false)) {
            double angleToTarget = getAngleToTarget.getDouble(1000);
            if (angleToTarget != lastAngleToTarget) {
                targetAngle = (drivetrain.getGyroDegrees() + angleToTarget) % 360;
                if (targetAngle < 0) {
                    targetAngle += 360;
                }
                drivetrain.setRotationPIDSetpoint(targetAngle);
            }
            lastAngleToTarget = angleToTarget;
        } else {
            lastAngleToTarget = 1000;
        }

        drivetrain.updateRotationPID(drivetrain.getGyroDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDriveMotors();
        visionSubsystem.setLightState(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        System.out.println("Angle: " + lastAngleToTarget + " Tolerance: " + Drivetrain.STEERING_ANGLE_TOLERANCE);
        System.out.println("Within PID :" + drivetrain.isRotationPIDWithinTolerance());
        return endWhenAligned && Math.abs(lastAngleToTarget) < Drivetrain.STEERING_ANGLE_TOLERANCE && drivetrain.isRotationPIDWithinTolerance();
    }
}
