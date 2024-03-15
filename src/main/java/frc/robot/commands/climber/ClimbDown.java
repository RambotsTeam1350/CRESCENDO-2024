package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
    private final Climber m_climber;

    public ClimbDown(Climber climber) {
        this.m_climber = climber;
        addRequirements(this.m_climber);
    }

    @Override
    public void initialize() {
        this.m_climber.setVoltageSetpoint(8 * Constants.Climber.UP_DIRECTION);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_climber.setVoltageSetpoint(0);
    }
}
