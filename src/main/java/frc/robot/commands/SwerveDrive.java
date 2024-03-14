/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;


/**
 * Drives the robot utilizing the full capabilities of swerve drive
 */
public class SwerveDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    boolean fieldRelative;

    private final Drivetrain drivetrain;

    private DoubleSupplier rightInput;
    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;

    /**
     * Creates a new SwerveDrive.
     *
     * @param drivetrain The subsystem used by this command.
     */
    public SwerveDrive(Drivetrain drivetrain, DoubleSupplier getRight, DoubleSupplier getForward, DoubleSupplier getRotation, boolean fieldRelative) {
        this.drivetrain = drivetrain;
        rightInput = getRight;
        forwardInput = getForward;
        rotationInput = getRotation;

        this.fieldRelative = fieldRelative;

        // Declare dependency on the drivetrain subsystem
        addRequirements(drivetrain);
    }

    public SwerveDrive(Drivetrain drivetrain, DoubleSupplier getRight, DoubleSupplier getForward, DoubleSupplier getRotation) {
        this(drivetrain, getRight, getForward, getRotation, true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double y = rightInput.getAsDouble();
        double x = forwardInput.getAsDouble();
        double z = rotationInput.getAsDouble();
        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(z) < 0.1) {
            drivetrain.stopDriveMotors();
        }

        // Create a WPILib object representing the desired velocity of the robot
        ChassisSpeeds desiredSpeeds;
        if (fieldRelative) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x * 2.0,
                    y * 2.0,
                    -z * Math.PI * 2,
                    drivetrain.getGyroRotation()
            );
        } else {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x * 2.0,
                    y * 2.0,
                    -z * Math.PI * 2,
                    Rotation2d.fromDegrees(0)
            );
        }


        drivetrain.setChassisSpeeds(desiredSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopAll();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
