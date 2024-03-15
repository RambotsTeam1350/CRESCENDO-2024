package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Intake;

public class RotateIntakeDownAndRunPowerMotorsInUntilNoteDetected extends Command {
    private final Intake m_intake;
    private final ColorSensor m_colorSensor;

    private boolean rotationMotorSlowed = false;

    public RotateIntakeDownAndRunPowerMotorsInUntilNoteDetected(Intake intake, ColorSensor colorSensor) {
        this.m_intake = intake;
        this.m_colorSensor = colorSensor;
    }

    @Override
    public void initialize() {
        this.m_intake.setRotationMotorVoltageSetpoint(12 * Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION);
        this.m_intake.setPowerMotorVelocitySetpoint(2400 * Constants.Intake.POWER_MOTOR_IN_DIRECTION);
    }

    @Override
    public void execute() {
        if (Math.abs(this.m_intake.getRotationAbsolutePosition() - Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE) <= 0.02
                && !this.rotationMotorSlowed) {
            this.m_intake.setRotationMotorVoltageSetpoint(6 * Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION);
            this.rotationMotorSlowed = true;
        } else if (Math.abs(
                this.m_intake.getRotationAbsolutePosition() - Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE) <= 0.005) {
            this.m_intake.stopRotationMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return this.m_colorSensor.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopRotationMotor();
        this.m_intake.stopPowerMotor();
    }
}
