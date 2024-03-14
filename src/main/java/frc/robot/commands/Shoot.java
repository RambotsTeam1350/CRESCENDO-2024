package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private Shooter m_shooter;
    private Intake m_intake;

    public Shoot(Shooter shooter, Intake intake) {
        this.m_shooter = shooter;
        this.m_intake = intake;
        addRequirements(this.m_shooter, this.m_intake);
    }

    @Override
    public void initialize() {
        this.m_intake.setPowerMotorVelocity(-10); // -10 RPM for testing purposes
        this.m_shooter.setMaxVelocitySetpoint();
    }

    @Override
    public void end(boolean isFinished) {
        this.m_shooter.stopSpeedMotors();
    }
}
