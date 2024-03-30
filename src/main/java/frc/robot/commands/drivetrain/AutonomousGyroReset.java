package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;

public class AutonomousGyroReset extends Command {
    private final Drivetrain drivetrainSubsystem;
    private final String kSide;

    public AutonomousGyroReset(Drivetrain drivetrainSubsystem, String side) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.kSide = side;
    }

    @Override
    public void initialize() {
        if (this.kSide.equals("Amp")) {
            if (!this.drivetrainSubsystem.isRedAlliance()) {
                this.drivetrainSubsystem.setHeading(60);
            } else {
                this.drivetrainSubsystem.setHeading(-60);
            }
        } else if (this.kSide.equals("Source")) {
            if (!this.drivetrainSubsystem.isRedAlliance()) {
                this.drivetrainSubsystem.setHeading(-60);
            } else {
                this.drivetrainSubsystem.setHeading(60);
            }
        }
    }
}
