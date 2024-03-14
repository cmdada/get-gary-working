package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gate;
import edu.wpi.first.wpilibj.Timer;

public class CloseGate extends Command {
    private Gate gate;
    private Timer timer = new Timer();

    public CloseGate(Gate gate) {
        this.gate = gate;
        addRequirements(gate);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        gate.closeGate();
    }

    @Override
    public void execute() {
        if (timer.get() >= 1) {
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
