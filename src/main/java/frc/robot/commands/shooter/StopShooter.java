package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
    private Shooter shooter;

    public StopShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        this.shooter.stopSpeedMotors();
    }
}
