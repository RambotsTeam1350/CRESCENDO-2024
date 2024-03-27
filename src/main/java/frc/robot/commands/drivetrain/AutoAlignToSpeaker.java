package frc.robot.commands.drivetrain;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Camera;

public class AutoAlignToSpeaker extends Command {
    private final Drivetrain drivetrainSubystem;
    private final Camera cameraSubsystem;

    private boolean hasDesiredTarget;
    private PhotonTrackedTarget target;

    public AutoAlignToSpeaker(Drivetrain drivetrainSubsystem, Camera cameraSubsystem) {
        this.drivetrainSubystem = drivetrainSubsystem;
        this.cameraSubsystem = cameraSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!this.cameraSubsystem.hasTarget()) {
            return;
        }
        for (PhotonTrackedTarget target : this.cameraSubsystem.getTargets()) {
            if (target.getFiducialId() == Constants.Vision.FiducialIDs.SPEAKER_BLUE
                    || target.getFiducialId() == Constants.Vision.FiducialIDs.SPEAKER_RED) {
                this.target = target;
                this.hasDesiredTarget = true;
            }
        }
        if (!this.hasDesiredTarget) {
            return;
        }
        this.drivetrainSubystem.rotateToFaceVisionTarget(this.cameraSubsystem.getBestTarget().getYaw());
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
