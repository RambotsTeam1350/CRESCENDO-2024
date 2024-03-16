package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;

public class FeedNote extends Command {
    private final Intake intake;

    public FeedNote(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        this.intake.setPowerMotorVelocitySetpoint(
                Constants.Intake.POWER_MOTOR_MAX_RPM * 0.75 * Constants.Intake.POWER_MOTOR_OUT_DIRECTION);
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.setPowerMotorVelocitySetpoint(0);
    }
}
