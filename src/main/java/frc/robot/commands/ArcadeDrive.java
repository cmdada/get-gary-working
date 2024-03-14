/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;


/**
 * Point the wheels straight forwards and drive like a standard differential drive.
 */
public class ArcadeDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain drivetrain;

    private DoubleSupplier joystickY;
    private DoubleSupplier joystickZ;

    /**
     * Creates a new ArcadeDrive.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArcadeDrive(Drivetrain subsystem, DoubleSupplier getJoystickY, DoubleSupplier getJoystickZ) {
        drivetrain = subsystem;
        joystickY = getJoystickY;
        joystickZ = getJoystickZ;

        //Declare dependency on the drivetrain subsystem
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //Point each wheel straight forwards
        for(Drivetrain.MotorLocation wheel : Drivetrain.MotorLocation.values()) {
            //drivetrain.setDesiredWheelAngle(wheel, 0);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.arcadeDrive(-joystickY.getAsDouble(), -joystickZ.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
