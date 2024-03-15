package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Intake;

public class RotateIntakeDownAndRunMotorsInUntilNoteGrabbed extends Command {
    private final Intake m_intake;
    private final ColorSensor m_colorSensor;

    private boolean rotationMotorSlowed = false;

    public RotateIntakeDownAndRunMotorsInUntilNoteGrabbed(Intake intake, ColorSensor colorSensor) {
        this.m_intake = intake;
        this.m_colorSensor = colorSensor;
    }

    @Override
    public void initialize() {
        this.m_intake.setRotationMotorVoltageSetpoint(9 * Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION);
        this.m_intake.setPowerMotorVelocitySetpointFF(4000 * Constants.Intake.POWER_MOTOR_IN_DIRECTION);
    }

    @Override
    public void execute() {
        if (Math.abs(this.m_intake.getRotationAbsoluteEncoder() - Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE) <= 0.10
                && !this.rotationMotorSlowed) {
            this.m_intake.setRotationMotorVoltageSetpoint(6 * Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION);
            this.rotationMotorSlowed = true;
        } else if (Math.abs(
                this.m_intake.getRotationAbsoluteEncoder() - Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE) <= 0.005) {
            this.m_intake.stopRotationMotor();
        }

        if (this.m_colorSensor.isNoteDetected()) {
            Commands.waitSeconds(1);
            this.m_intake.stopPowerMotor();
            Commands.waitSeconds(2);
            this.m_intake.setRotationMotorVoltageSetpoint(9 * Constants.Intake.ROTATION_MOTOR_UP_DIRECTION);
            if (Math.abs(
                    this.m_intake.getRotationAbsoluteEncoder() - Constants.Intake.UP_ABSOLUTE_ENCODER_VALUE) <= 0.10
                    && !this.rotationMotorSlowed) {
                this.m_intake.setRotationMotorVoltageSetpoint(6 * Constants.Intake.ROTATION_MOTOR_UP_DIRECTION);
                this.rotationMotorSlowed = true;
            } else if (Math.abs(
                    this.m_intake.getRotationAbsoluteEncoder() - Constants.Intake.UP_ABSOLUTE_ENCODER_VALUE) <= 0.005) {
                this.m_intake.stopRotationMotor();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopRotationMotor();
        this.m_intake.stopPowerMotor();
    }
}
