package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterRotation;

public class RotateShooterToAngle extends Command {
    private final double kAngle;

    private final ShooterRotation shooterRotation;

    public RotateShooterToAngle(ShooterRotation shooterRotation, double angle) {
        this.kAngle = angle;
        this.shooterRotation = shooterRotation;
        addRequirements(this.shooterRotation);
    }

    @Override
    public void initialize() {
        // uncomment if things go haywire when you try to set to a position it's already
        // at/very close to (SHOULD NOT BE NEEDED)

        // if (Math.abs(this.intakeRotation.getAngle() - this.kAngle) <= 5) {
        // this.end(false);
        // }
    }

    @Override
    public void execute() {
        this.shooterRotation.setAngle(this.kAngle);
    }

    @Override
    public boolean isFinished() {
        return this.shooterRotation.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterRotation.stopMotor();
    }

    public static Command createShooterUpCommand(ShooterRotation shooterRotation) {
        return new RotateShooterToAngle(shooterRotation, Constants.Shooter.MAXIMUM_DEGREES_UP);
    }

    public static Command createShooterDownCommand(ShooterRotation shooterRotation) {
        return new RotateShooterToAngle(shooterRotation, Constants.Shooter.MAXIMUM_DEGREES_DOWN);
    }
}
