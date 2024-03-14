package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gate;

public class OpenGate extends Command {
    private Gate gate;
    private Timer timer = new Timer();

    public OpenGate(Gate gate) {
        this.gate = gate;
        addRequirements(gate);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        gate.openGate();
    }

    @Override
    public void execute() {
        if (timer.get() >= 0.5) {
            timer.stop();
            timer.reset();
            gate.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        gate.stop();
    }
}
