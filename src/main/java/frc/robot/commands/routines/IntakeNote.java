package frc.robot.commands.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.intake.GrabNote;
import frc.robot.commands.intake.RotateIntakeToAngle;
import frc.robot.commands.intake.SlowGrabNote;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.LEDCANdle;
import frc.robot.subsystems.intake.IntakePower;
import frc.robot.subsystems.intake.IntakeRotation;

public class IntakeNote extends SequentialCommandGroup {
        public IntakeNote(IntakeRotation intakeRotationSubsystem, IntakePower intakePowerSubsystem,
                        ColorSensor colorSensorSubsystem, LEDCANdle ledCANdle) {
                this.addCommands(
                                // rotate intake down while running intake power motors until note gets detected
                                RotateIntakeToAngle.createIntakeDownCommand(intakeRotationSubsystem)
                                                .alongWith(new GrabNote(intakePowerSubsystem, colorSensorSubsystem)),
                                new InstantCommand(() -> ledCANdle.setAllToOrange()),
                                // run intake power motors for a bit to ensure note gets positioned correctly
                                new SlowGrabNote(intakePowerSubsystem).withTimeout(0.45),
                                // rotate intake back up
                                RotateIntakeToAngle.createIntakeUpCommand(intakeRotationSubsystem));
        }
}
