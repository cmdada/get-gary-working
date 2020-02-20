package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives to specific coordinates relative to the defined 0,0 point in Drivetrain.odometry
 */
public class GotoPosition extends CommandBase {
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
        // Update Odometry + get new pose
        Pose2d pose = drivetrain.getOdometry().update(
                drivetrain.getGyroRotation(),
                drivetrain.getSwerveState(Drivetrain.MotorLocation.FRONT_LEFT),
                drivetrain.getSwerveState(Drivetrain.MotorLocation.FRONT_RIGHT),
                drivetrain.getSwerveState(Drivetrain.MotorLocation.BACK_LEFT),
                drivetrain.getSwerveState(Drivetrain.MotorLocation.BACK_RIGHT)
        );

        // Update PID with new position
        double distanceToTarget = startPosition.minus(pose.getTranslation()).getNorm();
        double speed = pidController.calculate(distanceToTarget);

        // Set speed
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                unitVelocity.getX() * speed,
                unitVelocity.getY() * speed,
                0,
                drivetrain.getGyroRotation()
        );
        drivetrain.setChassisSpeeds(chassisSpeeds); // Apply speed
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
