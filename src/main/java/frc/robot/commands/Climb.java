package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
    // TODO: trapezoidal motion profile to lower the climber down slowly so our
    // robot doesn't break (THIS WILL PROBABLY NOT BE EASY I HAVE TO DO
    // RESEARCH - MYER)
    private final Climber m_climber;

    public Climb(Climber climber) {
        this.m_climber = climber;
        addRequirements(this.m_climber);
    }

    @Override
    public void initialize() {
        this.m_climber.setVoltageSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
