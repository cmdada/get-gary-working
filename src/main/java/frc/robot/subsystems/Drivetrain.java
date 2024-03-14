/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; //Note: add build/libs/FRC 2020 to your classpath.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {
    public static final double STEERING_ANGLE_TOLERANCE = 10; //The acceptable steering error for each wheel in degrees
    private final double DRIVE_ENCODER_COUNTS_PER_METER = 20 * 6.67 /(4 * 0.0254 * Math.PI);
    // 20 encoder counts/rot, 6.67:1 gear ratio, 4 inch wheel dia., 0.0254 meters per inch

    /**
     * Stores data for each swerve module.
     */
    public enum MotorLocation {
        FRONT_LEFT(
                0,
                new Translation2d(0.269875, -0.320675), // coordinates: Wheel coordinates in meters for odometry
                new WPI_VictorSPX(Constants.frontLeftDriveMotorNumber), // driveMotor: Reference to ESC for drive
                new WPI_TalonSRX(Constants.frontLeftSteeringMotorNumber), // steerMotor: Reference to ESC for steer
                new Encoder(0, 1) // driveEncoder: reference to encoder for drive CIM
        ),
        FRONT_RIGHT(
                1,
                new Translation2d(0.269875, 0.320675),
                new WPI_VictorSPX(Constants.frontRightDriveMotorNumber),
                new WPI_TalonSRX(Constants.frontRightSteeringMotorNumber),
                new Encoder(2, 3)
        ),
        BACK_LEFT(
                2,
                new Translation2d(-0.269875, -0.320675),
                new WPI_VictorSPX(Constants.backLeftDriveMotorNumber),
                new WPI_TalonSRX(Constants.backLeftSteeringMotorNumber),
                new Encoder(4, 5)
        ),
        BACK_RIGHT(
                3,
                new Translation2d(-0.269875, 0.320675),
                new WPI_VictorSPX(Constants.backRightDriveMotorNumber),
                new WPI_TalonSRX(Constants.backRightSteeringMotorNumber),
                new Encoder(6, 7)
        );

        public final int index;
        public final Translation2d coordinates;
        public final WPI_VictorSPX driveMotor;
        public final WPI_TalonSRX steeringMotor;
        public final Encoder driveEncoder;
        public boolean inverted;

        // Constructor for a MotorLocation object, used above
        MotorLocation(int index, Translation2d coords, WPI_VictorSPX driveMotor, WPI_TalonSRX steeringMotor, Encoder driveEncoder) {
            this.index = index;
            this.coordinates = coords;
            this.driveMotor = driveMotor;
            this.steeringMotor = steeringMotor;
            this.driveEncoder = driveEncoder;
            this.inverted = false;
        }
    }

    private final double PID_TOLERANCE = 3; // Tolerance for the bot heading pid in degrees
    private final PIDController rotationPIDController; // PID controller to rotate bot to target heading
    private final SwerveDriveKinematics kinematics; // Handles math to find target wheel angles and speeds
    private final SwerveDriveOdometry odometry; // Keeps track of robot position on field (probably unreliably)
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(); // Reference to the gyro on RoboRIO

    /**
     * Creates a new Drivetrain.
     */
    public Drivetrain() {
        //Configure the Talons for PID control
        for (MotorLocation loc : MotorLocation.values()) { // Loop through each swerve module
            WPI_TalonSRX talon = loc.steeringMotor;
            talon.configFactoryDefault(); // Reset all settings to default

            loc.driveMotor.configFactoryDefault();  // Reset the drive motor settings
            loc.driveMotor.setInverted(true); // Invert the drive motor for some reason
            loc.driveMotor.configOpenloopRamp(Constants.drivetrainAcceleration); // Limit the motor's acceleration

            Encoder encoder = loc.driveEncoder;
            encoder.setReverseDirection(true); // Invert the drive encoder for som reason
        }
        configForRelativeEncoders();

        //Create kinematics and odometry objects
        kinematics = new SwerveDriveKinematics(
                MotorLocation.FRONT_LEFT.coordinates,
                MotorLocation.FRONT_RIGHT.coordinates,
                MotorLocation.BACK_LEFT.coordinates,
                MotorLocation.BACK_RIGHT.coordinates
        );
        odometry = new SwerveDriveOdometry(
                kinematics,
                getGyroRotation(), // Starting rotation
                new Pose2d() // Default position 0,0 - set starting position on Shuffleboard?
        );

        rotationPIDController = new PIDController(0, 0, 0); // TODO: Calibrate heading pid
        rotationPIDController.setTolerance(PID_TOLERANCE);
        rotationPIDController.enableContinuousInput(0, 360);

        SmartDashboard.putData("Gyro", gyro);
    }

    public void configForRelativeEncoders() {
        for (MotorLocation loc : MotorLocation.values()) { // Loop through each motor encoder
            WPI_TalonSRX talon = loc.steeringMotor;
            talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            talon.config_kP(0, 10); // TODO Tune this better?
            talon.config_kD(0, 0);
            talon.config_kI(0, 0);
            talon.configAllowableClosedloopError(0, (int) (STEERING_ANGLE_TOLERANCE / 360.0 * Constants.relativePulsesPerRevolution));
            talon.setInverted(true);
        }
    }

    @Override
    public void periodic() {
        // Dump debug info to Shuffleboard
        SmartDashboard.putNumber("FL Angle", getWheelDegrees(MotorLocation.FRONT_LEFT));
        SmartDashboard.putNumber("FL Encoder",  MotorLocation.FRONT_LEFT.steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("FR Angle", getWheelDegrees(MotorLocation.FRONT_RIGHT));
        SmartDashboard.putNumber("FR Encoder",  MotorLocation.FRONT_RIGHT.steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("BL Angle", getWheelDegrees(MotorLocation.BACK_LEFT));
        SmartDashboard.putNumber("BL Encoder",  MotorLocation.BACK_LEFT.steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("BR Angle", getWheelDegrees(MotorLocation.BACK_RIGHT));
        SmartDashboard.putNumber("BR Encoder",  MotorLocation.BACK_RIGHT.steeringMotor.getSelectedSensorPosition());

        //SmartDashboard.putNumber("X", odometry.getPoseMeters().getTranslation().getX());
        //SmartDashboard.putNumber("Y", odometry.getPoseMeters().getTranslation().getY());

        // Update odometry with the angle and speed of each wheel
        getOdometry().update(
            getGyroRotation(),
            getSwerveState(Drivetrain.MotorLocation.FRONT_LEFT),
            getSwerveState(Drivetrain.MotorLocation.FRONT_RIGHT),
            getSwerveState(Drivetrain.MotorLocation.BACK_LEFT),
            getSwerveState(Drivetrain.MotorLocation.BACK_RIGHT)
        );
    }

    /**
     * Set the speed of an individual drive motor
     * @param motor the motor to control
     * @param speed the desired motor speed, from -1 to 1
     */
    public void setDriveMotorSpeed(MotorLocation motor, double speed) {
        // Clamp the speed from -1 to 1
        speed = Math.copySign(Math.min(Math.abs(speed), Constants.drivetrainMaxSpeed), speed);
        if (motor.inverted) {
            speed = -speed;
        }
        motor.driveMotor.set(speed);
    }

    /**
     * Set the speeds of the drive motors such that the robot moves as specified by the forward and rotation paramater
     * Assumes that the wheels are pointed forwards
     * @param forward The desired forwards speed, from -1 to 1
     * @param rotation The desired angular velocity, from -1 to 1
     */
    public void arcadeDrive(double forward, double rotation) {
        // Arcade drive only used for testing/emergencies
        double leftSpeed = forward - rotation;
        double rightSpeed = forward + rotation;

        double max = Math.max(1, Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed)));
        leftSpeed /= max;
        rightSpeed /= max;

        setDriveMotorSpeed(MotorLocation.FRONT_LEFT, leftSpeed);
        setDriveMotorSpeed(MotorLocation.BACK_LEFT, leftSpeed);
        setDriveMotorSpeed(MotorLocation.FRONT_RIGHT, rightSpeed);
        setDriveMotorSpeed(MotorLocation.BACK_RIGHT, rightSpeed);
    }

    /**
     * Takes the given ChassisSpeeds and applies them to the drive and steering motors
     * @param chassisSpeeds the ChassisSpeeds to use
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        // Calculates the desired state (speed and angle) for each swerve module
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
            // Set the drive motor speeds
            setDriveMotorSpeed(loc, moduleStates[loc.index].speedMetersPerSecond / 2.0);

            // Set the desired steering angles
            double rawAngle = moduleStates[loc.index].angle.getDegrees();
            setDesiredWheelAngle(loc, -rawAngle);
        }
    }

    /**
     * Stop the drive motors immediately
     */
    public void stopDriveMotors() {
        for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) { // Loop through each swerve module
            setDriveMotorSpeed(loc, 0);
        }
    }

    /**
     * Stop all of the motors immediately
     */
    public void stopAll() {
        stopDriveMotors();

        for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
            loc.steeringMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * Find the displacement between two angles with the smallest magnitude, assuming 0 degrees is equal to 360 degrees
     * @param angle1 The value of the initial angle, in degrees
     * @param angle2 THe value of the final angle, in degrees
     * @return The calculated displacement, in degrees
     */
    private double getAngleDifference(double angle1, double angle2) {
        if (Math.abs(angle2 - angle1) <= 360 - Math.abs(angle2 - angle1)) {
            return angle2 - angle1;
        } else {
            return Math.copySign(360 - Math.abs(angle1 - angle2), angle1 - angle2);
        }
    }

    /**
     * Calculate the angle of one of the swerve wheels based on its encoder position
     * @param loc The wheel to find the angle of
     * @return The calculated angle, in degrees, counterclockwise from the initial position, from 0 to 360
     */
    public double getWheelDegrees(MotorLocation loc) {
        WPI_TalonSRX talon = loc.steeringMotor; // The encoder can be accessed through the Talon object

        //Get the position of the encoder, in raw units
        int encoderTicks = talon.getSelectedSensorPosition();

        //Convert the encoder position to a wheel angle
        double angle =  (encoderTicks / Constants.relativePulsesPerRevolution * 360) % 360;
        if (angle < 0) {angle += 360;}
        return angle;

    }

    /**
     * Get the angle of the wheel as Rotation2d
     */
    public Rotation2d getWheelRotation(MotorLocation wheel) {
        return new Rotation2d(-getWheelDegrees(wheel)/180*Math.PI); // Idk why there's a '-'here
    }

    /**
     * Determine whether the specified wheel angle is within the tolerance STEERING_ANGLE_TOLERANCE
     *
     * @param wheel The wheel to check
     * @return Whether the wheel is within tolerance
     */
    private boolean isWheelWithinTolerance(MotorLocation wheel) {
        return Math.abs(wheel.steeringMotor.getClosedLoopError()) < STEERING_ANGLE_TOLERANCE / 360.0 * Constants.relativePulsesPerRevolution;
    }

    /**
     * Determine whether all of the wheels are within the tolerance STEERING_ANGLE_TOLERANCE
     *
     * @return Whether the wheels are within tolerance
     */
    public boolean areAllWheelsWithinTolerance() {
        return (
            isWheelWithinTolerance(MotorLocation.FRONT_LEFT) &&
            isWheelWithinTolerance(MotorLocation.FRONT_RIGHT) &&
            isWheelWithinTolerance(MotorLocation.BACK_LEFT) &&
            isWheelWithinTolerance(MotorLocation.BACK_RIGHT)
        );
    }

    /**
     * Instruct the talon's integrated PID loop to rotate the wheel to a specific angle
     * @param wheel The wheel to control
     * @param target_angle The desired angle, in degrees, clockwise from the initial position, from 0 to 360
     */
    public void setDesiredWheelAngle(MotorLocation wheel, double target_angle) {
        // Clamp angle from 0 to 360
        target_angle = target_angle % 360;
        if (target_angle < 0) {target_angle += 360;}

        WPI_TalonSRX talon = wheel.steeringMotor;
        double current_angle = getWheelDegrees(wheel);
        // Find the shortest path to the target angle
        double displacement = getAngleDifference(current_angle, target_angle);

        // If it's quicker, invert the drive motor and rotate to the opposite angle instead
        if (Math.abs(displacement) < 90) {
            wheel.inverted = false;
        } else {
            wheel.inverted = true;
            target_angle = (target_angle + 180) % 360;
            displacement = getAngleDifference(current_angle, target_angle);
        }

        // Set the pid setpoint relative the current encoder position. The actual pid loop runs on the talon itself
        int current_encoder_position = talon.getSelectedSensorPosition();
        double desiredEncoderPosition = current_encoder_position + displacement * Constants.relativePulsesPerRevolution / 360.0;
        talon.set(ControlMode.Position, desiredEncoderPosition);
    }

    /**
     * Angle the wheels perpendicular to the line connecting them to the center of the chassis, ideal for rotation in place
     */
    private void angleWheelsForRotation() {
        // Possibly used for autonomous or vision alignment

        // Cheat and have the kinematics figure out which way the wheels should point while rotating
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, -1);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
            // Set the desired steering angles
            double rawAngle = moduleStates[loc.index].angle.getDegrees();
            setDesiredWheelAngle(loc, -rawAngle); // Why '-' here?
        }
    }

    /**
     * Initialize the pid controller to rotate towards a desired heading. You must call updateRotationPID() to update it.
     */
    public void initRotationPID() {
        rotationPIDController.reset();
        angleWheelsForRotation(); // I kinda hate that this is lumped in with initializing the pid loop
    }

    /**
     * Configures the PID loop gains
     *
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     */
    public void configurePIDLoop(double p, double i, double d) {
        rotationPIDController.setP(p);
        rotationPIDController.setI(i);
        rotationPIDController.setD(d);
    }

    /**
     * Once the pid has been initialized, update the drive wheel speeds based on the current gyro angle
     *
     * @param currentAngle The current rotation of the robot, as measured by the gyro
     */
    public void updateRotationPID(double currentAngle) {
        currentAngle = currentAngle % 360;
        if (currentAngle < 0) { currentAngle += 360; }

        double rotationSpeedRaw = rotationPIDController.calculate(currentAngle);
        // Clamp the speed form -0.4 to 0.4
        double rotationSpeed = Math.copySign(Math.min(0.4, Math.abs(rotationSpeedRaw)), rotationSpeedRaw);

        for (MotorLocation motorLocation: MotorLocation.values()) { // Loop through the swerve modules
            // Only move the drive motors if the steering motors have reached their setpoint
            // Call areAllWheelsWithinTolerance() for each module because screw efficiency I don't feel like fixing it
            if (areAllWheelsWithinTolerance()) {
                setDriveMotorSpeed(motorLocation, rotationSpeed);
            } else {
                setDriveMotorSpeed(motorLocation, 0);
            }
        }
    }

    /**
     * Change the target angle of the rotation PID controller
     *
     * @param setpoint The new setpoint
     */
    public void setRotationPIDSetpoint(double setpoint) {
        rotationPIDController.setSetpoint(setpoint);
    }

    /**
     * Check whether the PID controller is within the tolerance PID_TOLERANCE
     *
     * @return whether the controller is within tolerance
     */
    public boolean isRotationPIDWithinTolerance() {
        return rotationPIDController.atSetpoint();
    }

    /**
     * Get the gyro position as clockwise degrees
     *
     * @return the gyro angle as clockwise degrees
     */
    public double getGyroDegrees() {
        return (gyro.getAngle() + 180) % 360;
    }

    /**
     * Get the gyro position as a Rotation2d.
     *
     * @return a Rotation2d representing the gyro position
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getGyroDegrees());
    }

    /**
     * Resets the gyro position and updates the odometry as well.
     */
    public void resetGyro() {
        gyro.reset();
        odometry.resetPosition(odometry.getPoseMeters(), getGyroRotation());
    }

    /**
     * Gets the SwerveDriveKinematics of the drivetrain.
     * @return the SwerveDriveKinematics for the drivetrain
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Gets the SwerveDriveOdometry of the drivetrain.
     * @return the SwerveDriveOdometry for the drivetrain.
     */
    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    /**
     * Gets the state of the swerve module
     * @param module the swerve module to get the state of
     * @return a SwerveModuleState representing the rotation and speed of the module
     */
    public SwerveModuleState getSwerveState(MotorLocation module) {
        return new SwerveModuleState(
                module.driveEncoder.getRate() / DRIVE_ENCODER_COUNTS_PER_METER,
                getWheelRotation(module)
        );
    }
}