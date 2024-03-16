package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Intake;

public class GrabNote extends Command {
    private final Intake intake;
    private final ColorSensor colorSensor;

    // CURRENTLY UNUSED
    public GrabNote(Intake intake, ColorSensor colorSensor) {
        this.intake = intake;
        this.colorSensor = colorSensor;
        addRequirements(this.intake, this.colorSensor);
    }

    @Override
    public void initialize() {
        this.intake.setPowerMotorVelocitySetpoint(4500 * Constants.Intake.POWER_MOTOR_IN_DIRECTION);
    }

    @Override
    public boolean isFinished() {
        return this.colorSensor.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.stopPowerMotor();
    }
}
