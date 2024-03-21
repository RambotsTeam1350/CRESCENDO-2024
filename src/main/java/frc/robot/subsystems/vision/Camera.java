package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Camera extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private boolean hasTarget = false;
    private PhotonTrackedTarget bestTarget;

    public Camera() {
        this.camera = new PhotonCamera(Constants.Vision.TARGET_CAMERA);
        this.camera.setPipelineIndex(Constants.Vision.PIPELINE);
    }

    @Override
    public void periodic() {
        this.latestResult = camera.getLatestResult();

        if (this.latestResult.hasTargets()) {
            this.bestTarget = this.latestResult.getBestTarget();
            this.hasTarget = true;
        } else {
            this.hasTarget = false;
        }
    }

    public PhotonPipelineResult getLatestResult() {
        return this.latestResult;
    }

    public PhotonTrackedTarget getBestTarget() {
        return this.bestTarget;
    }

    public int getTargetFiducialID() {
        return this.bestTarget.getFiducialId();
    }

    public boolean hasTarget() {
        return this.hasTarget;
    }
}
