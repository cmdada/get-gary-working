package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives to specific coordinates relative to the defined 0,0 point in Drivetrain.odometry
 */
public class GotoPosition extends Command {
    private Drivetrain drivetrain;
    private ProfiledPIDController pidController;

    Translation2d startPosition;
    Translation2d targetPosition;
    Translation2d unitVelocity;

    public GotoPosition(Drivetrain drivetrain, Translation2d target) {
        this.drivetrain = drivetrain;
        startPosition = drivetrain.getOdometry().getPoseMeters().getTranslation();
        targetPosition = target; //TODO Convert normal people position into stupid wpilib position

        Translation2d diff = targetPosition.minus(startPosition);
        unitVelocity = diff.div(diff.getNorm()); // Unit vector pointing at target from starting position
        pidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        pidController.setGoal(diff.getNorm()); // Set PID target to total distance to target
        pidController.reset(0, 0); //TODO PID should be based on distance to target
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Update PID with new position
        double distanceToTarget = startPosition.minus(drivetrain.getOdometry().getPoseMeters().getTranslation()).getNorm();
        double speed = pidController.calculate(distanceToTarget);

        // Set speeds
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                unitVelocity.getX() * speed,
                unitVelocity.getY() * speed,
                0,
                drivetrain.getGyroRotation()
        );
        drivetrain.setChassisSpeeds(chassisSpeeds); // Apply speeds
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDriveMotors();
    }

    @Override
    public boolean isFinished() {
        return pidController.atGoal();
    }
}
