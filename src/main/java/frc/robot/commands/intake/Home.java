package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Home extends Command {
    private final Intake m_intake;

    public Home(Intake intake) {
        this.m_intake = intake;
        addRequirements(this.m_intake);
    }

    @Override
    public void initialize() {
        this.m_intake.setRotationVoltageSetpoint(6);
    }

    @Override
    public boolean isFinished() {
        return this.m_intake.getRotationAbsoluteEncoder() <= 0.323;
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopRotationMotor();
        this.m_intake.resetRotationMotorEncoder();
    }
}
