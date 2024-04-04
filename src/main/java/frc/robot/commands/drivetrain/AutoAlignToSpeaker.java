package frc.robot.commands.drivetrain;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
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
        double cameraDistanceFromSpeaker = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.Vision.CAMERA_HEIGHT_METERS,
                Constants.Vision.Measurements.Speaker.APRIL_TAG_HEIGHT_METERS,
                Constants.Vision.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(speakerTarget.getPitch()));
        double shooterDistanceFromSpeaker = cameraDistanceFromSpeaker
                - Constants.Vision.CAMERA_DISTANCE_FROM_EDGE_OF_ROBOT_METERS;
        if (this.isInRange(shooterDistanceFromSpeaker)) {
            SmartDashboard.putNumber("Speaker Yaw", speakerTarget.getYaw());
            this.drivetrainSubystem.rotateToFaceVisionTarget(speakerTarget.getYaw());
        } else {
            this.cancel();
        }
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

    private boolean isInRange(double positionX) {
        return positionX <= Constants.Vision.MaxDistances.SPEAKER;
    }
}
