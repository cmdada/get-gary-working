package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class CalibrateEncoders extends Command {
    private Drivetrain drivetrain;
    private Timer timer = new Timer();
    private boolean done = false;

    public CalibrateEncoders(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.configForAbsoluteEncoders();
        timer.reset();
        timer.start();
        done = false;
    }

    @Override
    public void execute() {
        if (!done && timer.get() >= 0.25) {
            done = true;
            for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
                int absEncoderValue = loc.steeringMotor.getSelectedSensorPosition();
                System.out.println(loc.name() + ": " + absEncoderValue);
            }
            drivetrain.configForRelativeEncoders();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 0.5;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
