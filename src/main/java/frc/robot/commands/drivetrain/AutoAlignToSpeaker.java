package frc.robot.commands.drivetrain;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Camera;

public class AutoAlignToSpeaker extends Command {
    private final Drivetrain drivetrainSubystem;
    private final Camera cameraSubsystem;

    public AutoAlignToSpeaker(Drivetrain drivetrainSubsystem, Camera cameraSubsystem) {
        this.drivetrainSubystem = drivetrainSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        addRequirements(drivetrainSubsystem);
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
        SmartDashboard.putNumber("Speaker Yaw", speakerTarget.getYaw());
        this.drivetrainSubystem.rotateToFaceVisionTarget(speakerTarget.getYaw());
    }

    @Override
    public boolean isFinished() {
        return this.drivetrainSubystem.isAtHeadingSetpoint();
        // || this.cameraSubsystem.getSpeakerTarget() == null;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrainSubystem.stopModules();
    }
}
