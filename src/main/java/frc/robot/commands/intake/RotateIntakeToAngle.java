package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.IntakeRotation;

public class RotateIntakeToAngle extends Command {
    private final double kAngle;

    private final IntakeRotation intakeRotation;

    public RotateIntakeToAngle(IntakeRotation intakeRotation, double angle) {
        this.kAngle = angle;
        this.intakeRotation = intakeRotation;
        addRequirements(this.intakeRotation);
    }

    @Override
    public void initialize() {
        // uncomment if things go haywire when you try to set to a position it's already
        // at/very close to

        // if (Math.abs(this.intakeRotation.getAngle() - this.kAngle) <= 5) {
        // this.end(false);
        // }
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
        System.out.println("RotateIntakeToAngle STOPPING");
        this.intakeRotation.stopMotor();
    }

    public static RotateIntakeToAngle createIntakeDownCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.DOWN_DEGREES);
    }

    public static RotateIntakeToAngle createIntakeUpCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.UP_DEGREES);
    }

    public static RotateIntakeToAngle createIntakeStraightCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.STRAIGHT_DEGREES);
    }
}
