package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private Shooter m_shooter;
    private Intake m_intake;

    public RunShooter(Shooter shooter, Intake intake) {
        this.m_shooter = shooter;
        this.m_intake = intake;
        addRequirements(this.m_shooter, this.m_intake);
    }

    @Override
    public void initialize() {
        this.m_shooter.setVelocitySetpointFF(4000);
    }

    @Override
    public void end(boolean isFinished) {
        this.m_intake.stopPowerMotor();
        this.m_shooter.stopSpeedMotors();
    }
}
