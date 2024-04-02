package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.intake.IntakePower;

public class GrabNote extends Command {
    private final IntakePower intakePower;
    private final ColorSensor colorSensor;

    public GrabNote(IntakePower intakePower, ColorSensor colorSensor) {
        this.intakePower = intakePower;
        this.colorSensor = colorSensor;
        addRequirements(this.intakePower, this.colorSensor);
    }

    @Override
    public void initialize() {
        this.intakePower.setMotorVelocitySetpoint(
                Constants.Intake.POWER_MOTOR_MAX_RPM * 0.4 * Constants.Intake.POWER_MOTOR_IN_DIRECTION);
    }

    @Override
    public boolean isFinished() {
        return this.colorSensor.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakePower.stopMotor();
    }
}
