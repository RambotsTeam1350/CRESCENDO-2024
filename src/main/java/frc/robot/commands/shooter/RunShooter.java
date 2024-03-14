package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private Shooter m_shooter;

    public RunShooter(Shooter shooter) {
        this.m_shooter = shooter;
        addRequirements(this.m_shooter);
    }

    @Override
    public void initialize() {
        this.m_shooter.setVelocitySetpointFF(4000);
    }

    @Override
    public void end(boolean isFinished) {
        this.m_shooter.stopSpeedMotors();
    }
}
