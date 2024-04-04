package frc.robot.commands.routines;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AutoAlignToSpeaker;
import frc.robot.commands.shooter.AutoRotateShooterToSpeakerAngle;
import frc.robot.subsystems.LEDCANdle;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Camera;

public class PrepareToShoot extends SequentialCommandGroup {
        public PrepareToShoot(ShooterRotation shooterRotationSubsystem, Camera cameraSubsystem,
                        Drivetrain drivetrainSubsystem, LEDCANdle ledCANdleSubsystem) {
                this.addCommands(
                                new InstantCommand(() -> ledCANdleSubsystem.toggleTaken()),
                                new AutoRotateShooterToSpeakerAngle(shooterRotationSubsystem, cameraSubsystem,
                                                ledCANdleSubsystem)
                                                .alongWith(new AutoAlignToSpeaker(drivetrainSubsystem, cameraSubsystem)
                                                                .withTimeout(5)),
                                // new ScheduleCommand(new InstantCommand(() ->
                                // ledCANdleSubsystem.setAllToOrange(),
                                // ledCANdleSubsystem).withTimeout(5)),
                                new InstantCommand(() -> ledCANdleSubsystem.toggleTaken()));
                addRequirements(ledCANdleSubsystem);
        }
}
