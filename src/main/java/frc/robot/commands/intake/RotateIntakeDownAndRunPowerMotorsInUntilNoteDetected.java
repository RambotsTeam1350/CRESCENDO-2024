package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Intake;

public class RotateIntakeDownAndRunPowerMotorsInUntilNoteDetected extends Command {
    private final Intake intake;
    private final ColorSensor colorSensor;

    private boolean rotationMotorSlowed = false;

    public RotateIntakeDownAndRunPowerMotorsInUntilNoteDetected(Intake intake, ColorSensor colorSensor) {
        this.intake = intake;
        this.colorSensor = colorSensor;
    }

    @Override
    public void initialize() {
        this.intake.setRotationMotorVoltageSetpoint(12 * Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION);
        this.intake.setPowerMotorVelocitySetpoint(
                Constants.Intake.POWER_MOTOR_MAX_RPM * 0.42 * Constants.Intake.POWER_MOTOR_IN_DIRECTION);
    }

    @Override
    public void execute() {
        if (Math.abs(
                this.intake.getRotationAbsolutePosition() - Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE) <= 0.015
                && !this.rotationMotorSlowed) {
            this.intake.setRotationMotorVoltageSetpoint(6 * Constants.Intake.ROTATION_MOTOR_DOWN_DIRECTION);
            this.rotationMotorSlowed = true;
        } else if (Math.abs(
                this.intake.getRotationAbsolutePosition() - Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE) <= 0.005) {
            this.intake.stopRotationMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return this.colorSensor.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.stopRotationMotor();
        this.intake.stopPowerMotor();
    }
}
