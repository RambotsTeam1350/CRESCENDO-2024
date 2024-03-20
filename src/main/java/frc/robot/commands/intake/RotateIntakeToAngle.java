package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.IntakeRotation;

public class RotateIntakeToAngle extends Command {
    private final double kAngle;
    private double kDirection;

    private final IntakeRotation intakeRotation;

    private boolean slowed = false;

    public RotateIntakeToAngle(IntakeRotation intakeRotation, double angle) {
        this.kAngle = angle;
        this.intakeRotation = intakeRotation;
        addRequirements(this.intakeRotation);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.intakeRotation.setAngle(this.kAngle);
    }

    @Override
    public boolean isFinished() {
        return this.intakeRotation.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeRotation.stopMotor();
    }

    public static RotateIntakeToAngle createIntakeDownCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE);
    }

    public static RotateIntakeToAngle createIntakeUpCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.UP_DEGREES);
    }

    public static RotateIntakeToAngle createIntakeStraightCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.STRAIGHT_ABSOLUTE_ENCODER_VALUE);
    }
}
