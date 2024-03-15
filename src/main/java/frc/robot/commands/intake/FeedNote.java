package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;

public class FeedNote extends Command {
    private final Intake m_intake;

    public FeedNote(Intake intake) {
        this.m_intake = intake;
    }

    @Override
    public void initialize() {
        this.m_intake.setPowerMotorVelocitySetpoint(100 * Constants.Intake.POWER_MOTOR_OUT_DIRECTION);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.setPowerMotorVelocitySetpoint(0);
    }
}
