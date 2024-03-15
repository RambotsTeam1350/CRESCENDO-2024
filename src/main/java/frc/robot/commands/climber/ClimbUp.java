package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climber;

public class ClimbUp extends Command {
    private final Climber m_climber;

    public ClimbUp(Climber climber) {
        this.m_climber = climber;
        addRequirements(this.m_climber);
    }

    @Override
    public void initialize() {
        this.m_climber.setVoltageSetpoint(16 * Constants.Climber.DOWN_DIRECTION);
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
