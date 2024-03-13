package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private Shooter m_shooter;

    public Shoot(Shooter shooter) {
        this.m_shooter = shooter;
        addRequirements(this.m_shooter);
    }

    @Override
    public void initialize() {
        this.m_shooter.setMaxVelocitySetpoint();
    }

    @Override
    public void end(boolean isFinished) {
        this.m_shooter.stopSpeedMotors();
    }
}
