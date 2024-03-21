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
        // at/very close to

        // if (Math.abs(this.intakeRotation.getAngle() - this.kAngle) <= 5) {
        // this.end(false);
        // }
    }

    @Override
    public void execute() {
        // this.intakeRotation.setAngle(this.kAngle);
    }

    // @Override
    // public boolean isFinished() {
    // // return this.intakeRotation.atSetpoint();
    // }

    @Override
    public void end(boolean interrupted) {
        this.shooterRotation.stopMotor();
    }
}
