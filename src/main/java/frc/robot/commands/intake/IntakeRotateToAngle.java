package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeRotateToAngle extends Command {
    private final double kPosition;
    private final int kDirection;

    private final Intake m_intake;

    public IntakeRotateToAngle(Intake intake, double position) {
        this.kPosition = position;
        this.m_intake = intake;

        this.kDirection = this.m_intake.getRotationAbsoluteEncoder() > kPosition ? -1 : 1;
        addRequirements(this.m_intake);
    }

    @Override
    public void initialize() {
        this.m_intake.setRotationMotorVoltageSetpoint(12 * this.kDirection);
        // this.m_intake.setRotationAngleSetpoint(Constants.Intake.DOWN_DEGREES); //
        // lower intake
    }

    @Override
    public void execute() {
        // once it gets close enough to position, slow down to hone in on position
        if (Math.abs(this.m_intake.getRotationAbsoluteEncoder() - kPosition) <= 0.15)
            this.m_intake.setRotationMotorVoltageSetpoint(6 * this.kDirection);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.m_intake.getRotationAbsoluteEncoder() - kPosition) <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopRotationMotor();
    }
}
