package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Intake;

public class ForceSlowGrabNote extends Command {
    private final Intake m_intake;

    public ForceSlowGrabNote(Intake intake) {
        this.m_intake = intake;
        addRequirements(this.m_intake);
    }

    @Override
    public void initialize() {
        this.m_intake.setPowerMotorVelocitySetpoint(240 * Constants.Intake.POWER_MOTOR_IN_DIRECTION);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopPowerMotor();
    }
}
