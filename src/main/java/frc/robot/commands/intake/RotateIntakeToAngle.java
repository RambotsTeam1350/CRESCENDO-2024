package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;

public class RotateIntakeToAngle extends Command {
    private final double kPosition;
    private double kDirection;

    private final Intake intake;

    private boolean slowed = false;

    public RotateIntakeToAngle(Intake intake, double position) {
        this.kPosition = position;
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.kDirection = this.intake.getRotationAbsolutePosition() > kPosition
                ? Constants.Intake.ROTATION_MOTOR_UP_DIRECTION
                : Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION;
        this.intake.setRotationMotorVoltageSetpoint(12 * this.kDirection);
        // this.m_intake.setRotationAngleSetpoint(Constants.Intake.DOWN_DEGREES); //
        // lower intake
    }

    @Override
    public void execute() {
        // once it gets close enough to position, slow down to hone in on position
        if (Math.abs(this.intake.getRotationAbsolutePosition() - kPosition) <= 0.015 && !this.slowed) {
            this.intake.setRotationMotorVoltageSetpoint(6 * this.kDirection);
            this.slowed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.intake.getRotationAbsolutePosition() - kPosition) <= 0.005;
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.stopRotationMotor();
    }
}
