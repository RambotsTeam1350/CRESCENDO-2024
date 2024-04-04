package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Camera extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private boolean hasTarget = false;
    private PhotonTrackedTarget bestTarget;
    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget speakerTarget;

    // private final AprilTagFieldLayout aprilTagFieldLayout;
    // private final PhotonPoseEstimator photonPoseEstimator;

    public Camera() {
        this.camera = new PhotonCamera(Constants.Vision.TARGET_CAMERA);
        this.camera.setPipelineIndex(Constants.Vision.PIPELINE);

        // this.aprilTagFieldLayout =
        // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera,
        // Constants.Vision.ROBOT_TO_CAM_TRANSFORM);
    }

    @Override
    public void periodic() {
        this.latestResult = camera.getLatestResult();

        if (this.latestResult.hasTargets()) {
            this.bestTarget = this.latestResult.getBestTarget();
            this.targets = this.latestResult.getTargets();
            this.speakerTarget = this.findSpeakerTarget();
            this.hasTarget = true;
        } else {
            this.hasTarget = false;
        }

        SmartDashboard.putBoolean("Has Speaker Target", this.hasSpeakerTarget());
    }

    public PhotonPipelineResult getLatestResult() {
        return this.latestResult;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return this.targets;
    }

    public PhotonTrackedTarget getSpeakerTarget() {
        return this.speakerTarget;
    }

    private PhotonTrackedTarget findSpeakerTarget() {
        if (this.targets == null) {
            return null;
        }
        for (PhotonTrackedTarget target : this.targets) {
            if (target.getFiducialId() == Constants.Vision.FiducialIDs.SPEAKER_BLUE
                    || target.getFiducialId() == Constants.Vision.FiducialIDs.SPEAKER_RED) {
                return target;
            }
        }
        return null;
    }

    public boolean hasSpeakerTarget() {
        return this.getSpeakerTarget() != null;
    }

    public PhotonTrackedTarget getBestTarget() {
        return this.bestTarget;
    }

    public boolean hasTarget() {
        return this.hasTarget;
    }

    // public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    // return this.photonPoseEstimator.update();
    // }
}
