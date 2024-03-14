/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

/**
 * Rotate in place until the gyro reading matches a specified value
 */
public class RotateToAngle extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain drivetrain;
    private double targetAngle;

    /**
     * Creates a new RotateToAngle command.
     *
     * @param subsystem The subsystem used by this command.
     * @param targetAngleParam The desired angle
     */
    public RotateToAngle(Drivetrain subsystem, double targetAngleParam) {
        drivetrain = subsystem;
        targetAngle = targetAngleParam;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.configurePIDLoop(-0.05, 0, -0.002);
        drivetrain.initRotationPID();
        drivetrain.setRotationPIDSetpoint(targetAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.updateRotationPID(drivetrain.getGyroDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDriveMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.isRotationPIDWithinTolerance();
    }
}
