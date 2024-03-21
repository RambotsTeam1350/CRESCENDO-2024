// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.LED;
import edu.wpi.first.math.util.Units;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TargetVision extends SubsystemBase {
  private PhotonCamera camera;
  public static TargetVision instance;
  private boolean hasTarget = false;
  PhotonTrackedTarget target;
  TargetVision m_cam = new TargetVision();
  LED m_led = new LED();

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(105);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(29.7);// 29.5

  private double rotationAngle;
  private double positionX;
  private double maxDistance = 1.7; // meters

  private int desiredFiducialID = 4;
  // the id currently seen by the camera
  private int tagID;

  public TargetVision() {

    this.camera = new PhotonCamera(Constants.targetCamera);
    this.camera.setPipelineIndex(Constants.pipeline);

  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      target = result.getBestTarget();

      positionX = PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(result.getBestTarget().getPitch()));

      if (m_cam.hasTargets() && m_cam.isInRange()) {
        m_led.setLEDs();
      }

      this.hasTarget = true;

    } else {
      this.hasTarget = false;
    }

  }

  public double getRotationAngle() {
    return rotationAngle = Math.atan((1.5) / positionX);
  }

  public int getTagID() {
    return tagID = target.getFiducialId();
  }

  public boolean hasTargets() {
    return this.hasTarget;
  }

  public double getPositionX() {
    return positionX;
  }

  public boolean isInRange() {
    if ((tagID == desiredFiducialID) && (positionX <= maxDistance))
      return true;

    return false;
  }

}
