package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.IntakePower;

public class ForceSlowGrabNote extends Command {
    private final IntakePower intakePowerSubsystem;

    public ForceSlowGrabNote(IntakePower intakePower) {
        this.intakePowerSubsystem = intakePower;
        addRequirements(this.intakePowerSubsystem);
        this.runsWhenDisabled();
    }

    @Override
    public void initialize() {
        this.intakePowerSubsystem.setMotorVelocitySetpoint(240 * Constants.Intake.POWER_MOTOR_IN_DIRECTION);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.intakePowerSubsystem.stopMotor();
    }
}
