package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;

public class Climb extends Command {
    private Climber climber;
    private BooleanSupplier getDirection;


    public Climb(Climber climber, BooleanSupplier getDirection) {
        this.climber = climber;
        this.getDirection = getDirection;
    }

    @Override
    public void execute() {
        if (getDirection.getAsBoolean()) {
            climber.up();
        }
        else {
            climber.down();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
