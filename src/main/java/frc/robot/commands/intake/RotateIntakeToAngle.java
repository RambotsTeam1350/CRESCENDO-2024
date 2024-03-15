package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;

public class RotateIntakeToAngle extends Command {
    private final double kPosition;
    private double kDirection;

    private final Intake m_intake;

    private boolean slowed = false;

    public RotateIntakeToAngle(Intake intake, double position) {
        this.kPosition = position;
        this.m_intake = intake;
        addRequirements(this.m_intake);
    }

    @Override
    public void initialize() {
        this.kDirection = this.m_intake.getRotationAbsoluteEncoder() > kPosition
                ? Constants.Intake.ROTATION_MOTOR_UP_DIRECTION
                : Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION;
        this.m_intake.setRotationMotorVoltageSetpoint(12 * this.kDirection);
        // this.m_intake.setRotationAngleSetpoint(Constants.Intake.DOWN_DEGREES); //
        // lower intake
    }

    @Override
    public void execute() {
        // once it gets close enough to position, slow down to hone in on position
        System.out.println(Math.abs(this.m_intake.getRotationAbsoluteEncoder() - kPosition));
        if (Math.abs(this.m_intake.getRotationAbsoluteEncoder() - kPosition) <= 0.10 && !this.slowed) {
            this.m_intake.setRotationMotorVoltageSetpoint(6 * this.kDirection);
            this.slowed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.m_intake.getRotationAbsoluteEncoder() - kPosition) <= 0.005;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopRotationMotor();
    }
}
