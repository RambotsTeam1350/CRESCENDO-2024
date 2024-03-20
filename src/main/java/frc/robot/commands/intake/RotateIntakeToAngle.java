package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.IntakeRotation;

public class RotateIntakeToAngle extends Command {
    private final double kPosition;
    private double kDirection;

    private final IntakeRotation intake;

    private boolean slowed = false;

    public RotateIntakeToAngle(IntakeRotation intakeRotation, double position) {
        this.kPosition = position;
        this.intake = intakeRotation;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.kDirection = this.intake.getEncoderAbsolutePosition() > kPosition
                ? Constants.Intake.ROTATION_MOTOR_UP_DIRECTION
                : Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION;
        this.intake.setMotorVelocitySetpoint(3500 * this.kDirection);
        // this.m_intake.setRotationAngleSetpoint(Constants.Intake.DOWN_DEGREES); //
        // lower intake
    }

    @Override
    public void execute() {
        // once it gets close enough to position, slow down to hone in on position
        if (Math.abs(this.intake.getEncoderAbsolutePosition() - kPosition) <= 0.1 && !this.slowed) {
            this.intake.setMotorVelocitySetpoint(250 * this.kDirection);
            this.slowed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.intake.getEncoderAbsolutePosition() - kPosition) <= 0.005;
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.stopMotor();
    }

    public static RotateIntakeToAngle createIntakeDownCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE);
    }

    public static RotateIntakeToAngle createIntakeUpCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.UP_ABSOLUTE_ENCODER_VALUE);
    }

    public static RotateIntakeToAngle createIntakeStraightCommand(IntakeRotation intakeRotation) {
        return new RotateIntakeToAngle(intakeRotation, Constants.Intake.STRAIGHT_ABSOLUTE_ENCODER_VALUE);
    }
}
