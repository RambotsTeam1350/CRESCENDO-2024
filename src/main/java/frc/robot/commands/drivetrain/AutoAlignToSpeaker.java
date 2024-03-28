package frc.robot.commands.drivetrain;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Camera;

public class AutoAlignToSpeaker extends Command {
    private final Drivetrain drivetrainSubystem;
    private final Camera cameraSubsystem;

    public AutoAlignToSpeaker(Drivetrain drivetrainSubsystem, Camera cameraSubsystem) {
        this.drivetrainSubystem = drivetrainSubsystem;
        this.cameraSubsystem = cameraSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        PhotonTrackedTarget speakerTarget = this.cameraSubsystem.getSpeakerTarget();
        if (speakerTarget == null) {
            return;
        }
        this.drivetrainSubystem.rotateToFaceVisionTarget(speakerTarget.getYaw());
    }

    @Override
    public boolean isFinished() {
        return this.drivetrainSubystem.isAtHeadingSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrainSubystem.stopModules();
    }
}
