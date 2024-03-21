package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterPower;

public class RunStopShooter extends Command {
    private ShooterPower shooterPower;

    public RunStopShooter(ShooterPower shooter) {
        this.shooterPower = shooter;
        addRequirements(this.shooterPower);
    }

    @Override
    public void initialize() {
        this.shooterPower.setMotorsVelocitySetpoint(Constants.Shooter.SPEED_MOTORS_MAX_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterPower.stopSpeedMotors();
    }
}