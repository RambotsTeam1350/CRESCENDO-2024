package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPower;

public class StopShooter extends Command {
    private ShooterPower shooterPower;

    public StopShooter(ShooterPower shooterPower) {
        this.shooterPower = shooterPower;
        addRequirements(this.shooterPower);
    }

    @Override
    public void initialize() {
        this.shooterPower.stopSpeedMotors();
    }
}
