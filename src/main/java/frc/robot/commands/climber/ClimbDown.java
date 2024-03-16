package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
    private final Climber climber;

    public ClimbDown(Climber climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void initialize() {
        this.climber.setVoltageSetpoint(8 * Constants.Climber.UP_DIRECTION);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.climber.setVoltageSetpoint(0);
    }
}
