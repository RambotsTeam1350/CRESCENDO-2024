package frc.robot.commands.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.FeedNote;
import frc.robot.commands.shooter.RunStopShooter;
import frc.robot.subsystems.intake.IntakePower;
import frc.robot.subsystems.shooter.ShooterPower;

public class ShootNote extends ParallelCommandGroup {
    public ShootNote(ShooterPower shooterPowerSubsystem, IntakePower intakePowerSubsystem) {
        this.addCommands(new RunStopShooter(shooterPowerSubsystem).withTimeout(2.75),
                Commands.waitSeconds(1.5).andThen(new FeedNote(intakePowerSubsystem).withTimeout(1.5 + 0.5)));
    }
}
