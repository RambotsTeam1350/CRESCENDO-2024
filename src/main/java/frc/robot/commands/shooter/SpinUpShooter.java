package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends Command {
    private Shooter shooter;

    public SpinUpShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        this.shooter.setSpeedMotorsVelocitySetpoint(Constants.Shooter.SPEED_MOTORS_MAX_RPM);
    }
}
