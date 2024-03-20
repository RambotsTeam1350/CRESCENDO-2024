package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.IntakePower;

public class FeedNote extends Command {
    private final IntakePower intakePowerSubsystem;

    public FeedNote(IntakePower intakePower) {
        this.intakePowerSubsystem = intakePower;
        addRequirements(this.intakePowerSubsystem);
    }

    @Override
    public void initialize() {
        this.intakePowerSubsystem.setMotorVelocitySetpoint(
                Constants.Intake.POWER_MOTOR_MAX_RPM * 0.65 * Constants.Intake.POWER_MOTOR_OUT_DIRECTION);
    }

    @Override
    public void end(boolean interrupted) {
        this.intakePowerSubsystem.setMotorVelocitySetpoint(0);
    }
}
