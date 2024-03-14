/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;
import java.util.function.DoubleSupplier;


/**
 * Command to test swerve functionality
 */
public class SwerveTest extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Drivetrain drivetrain;

    private DoubleSupplier rightInput;
    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;

    /**
     * Creates a new SwerveTest.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveTest(Drivetrain subsystem, DoubleSupplier getRight, DoubleSupplier getForward, DoubleSupplier getRotation) {
        drivetrain = subsystem;
        rightInput = getRight;
        forwardInput = getForward;
        rotationInput = getRotation;

        //Declare dependency on the drivetrain subsystem
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angle = rotationInput.getAsDouble() * 180;
        if (angle < 0) {angle += 360;}
        SmartDashboard.putNumber("Target Angle", angle);
        for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
            drivetrain.setDesiredWheelAngle(loc, angle);
        }
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
