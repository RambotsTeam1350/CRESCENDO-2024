// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.vision.Camera;

public class AutoRotateShooterToSpeakerAngle extends Command {
  private final ShooterRotation shooterRotation;
  private final Camera camera;
  private final LED led;

  private boolean hasDesiredTarget;
  private PhotonTrackedTarget target;

  public AutoRotateShooterToSpeakerAngle(ShooterRotation shooterRotation, Camera camera, LED led) {
    this.shooterRotation = shooterRotation;
    this.camera = camera;
    this.led = led;
    addRequirements(this.shooterRotation);
    // , this.camera, this.led); i am going to experiment with this
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("test");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!this.camera.hasTarget()) {
      return;
    }
    for (PhotonTrackedTarget target : this.camera.getTargets()) {
      if (target.getFiducialId() == Constants.Vision.FiducialIDs.SPEAKER_BLUE
          || target.getFiducialId() == Constants.Vision.FiducialIDs.SPEAKER_RED) {
        this.target = target;
        this.hasDesiredTarget = true;
      }
    }
    if (!this.hasDesiredTarget) {
      return;
    }
    double cameraDistanceFromSpeaker = PhotonUtils.calculateDistanceToTargetMeters(
        Constants.Vision.CAMERA_HEIGHT_METERS,
        Constants.Vision.Measurements.Speaker.APRIL_TAG_HEIGHT_METERS,
        Constants.Vision.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(this.target.getPitch()));
    double shooterDistanceFromSpeaker = cameraDistanceFromSpeaker
        - Constants.Vision.CAMERA_DISTANCE_FROM_EDGE_OF_ROBOT_METERS;
    SmartDashboard.putNumber("target yaw", this.target.getYaw());
    // System.out.println("POSITION X: " + cameraDistanceFromSpeaker);
    if (this.isInRange(shooterDistanceFromSpeaker)) {
      double angle = Units.radiansToDegrees(
          Math.atan(
              Constants.Vision.Measurements.Speaker.SHOOTER_TO_GOAL_HEIGHT_METERS / shooterDistanceFromSpeaker));
      angle -= Constants.Shooter.MAXIMUM_DEGREES_DOWN_ZERO_OFFSET;
      // angle = 3;
      System.out.println("SHOOTER ANGLE SETPOINT: " + angle);
      this.shooterRotation.setAngle(angle);
      this.led.setLEDs();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterRotation.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.shooterRotation.atSetpoint();
  }

  private boolean isInRange(double positionX) {
    return positionX <= Constants.Vision.MaxDistances.SPEAKER;
  }
}