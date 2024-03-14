package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class FeedNote extends Command {
    private final Intake m_intake;

    public FeedNote(Intake intake) {
        this.m_intake = intake;
    }

    @Override
    public void initialize() {
        this.m_intake.setPowerMotorVelocitySetpointFF(-10);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.setPowerMotorVelocitySetpointFF(0);
    }
}
