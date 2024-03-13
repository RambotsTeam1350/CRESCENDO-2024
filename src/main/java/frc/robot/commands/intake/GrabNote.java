package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Intake;

public class GrabNote extends Command {
    private final Intake m_intake;
    private final ColorSensor m_colorSensor;

    public GrabNote(Intake intake, ColorSensor colorSensor) {
        this.m_intake = intake;
        this.m_colorSensor = colorSensor;
        addRequirements(this.m_intake, this.m_colorSensor);
    }

    @Override
    public void initialize() {
        this.m_intake.setIntakeVelocitySetpoint(10); // 10 RPM for testing purposes
    }

    @Override
    public boolean isFinished() {
        return this.m_colorSensor.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        // this.m_intake.setIntakeVelocitySetpoint(0);
        this.m_intake.stopIntakeMotor(); // in case pid doesn't stop it (BIG PROBLEM IF TRUE!)
    }
}
