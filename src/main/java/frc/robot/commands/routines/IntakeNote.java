package frc.robot.commands.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.GrabNote;
import frc.robot.commands.intake.RotateIntakeToAngle;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.intake.IntakePower;
import frc.robot.subsystems.intake.IntakeRotation;

public class IntakeNote extends SequentialCommandGroup {
        public IntakeNote(IntakeRotation intakeRotationSubsystem, IntakePower intakePowerSubsystem,
                        ColorSensor colorSensorSubsystem) {
                this.addCommands(
                                // rotate intake down while running intake power motors until note gets detected
                                RotateIntakeToAngle.createIntakeDownCommand(intakeRotationSubsystem)
                                                .alongWith(new GrabNote(intakePowerSubsystem, colorSensorSubsystem)),
                                // run intake power motors for a bit to ensure note gets positioned correctly
                                Commands.waitSeconds(0.075),
                                // rotate intake back up
                                RotateIntakeToAngle.createIntakeUpCommand(intakeRotationSubsystem));
        }
}
