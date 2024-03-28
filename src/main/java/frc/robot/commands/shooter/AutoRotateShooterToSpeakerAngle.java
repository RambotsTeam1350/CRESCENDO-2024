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
  private final ShooterRotation shooterRotationSubsystem;
  private final Camera cameraSubsystem;
  private final LED led;

  public AutoRotateShooterToSpeakerAngle(ShooterRotation shooterRotation, Camera camera, LED ledSubsystem) {
    this.shooterRotationSubsystem = shooterRotation;
    this.cameraSubsystem = camera;
    this.led = ledSubsystem;
    addRequirements(this.shooterRotationSubsystem);
    // , this.camera, this.led); i am going to experiment with this
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
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
    // System.out.println("POSITION X: " + cameraDistanceFromSpeaker);
    if (this.isInRange(shooterDistanceFromSpeaker)) {
      double angle = Units.radiansToDegrees(
          Math.atan(
              Constants.Vision.Measurements.Speaker.SHOOTER_TO_GOAL_HEIGHT_METERS / shooterDistanceFromSpeaker));
      // angle -= Constants.Shooter.MAXIMUM_DEGREES_DOWN_ZERO_OFFSET;
      // System.out.println("SHOOTER ANGLE SETPOINT: " + angle); TODO: pls do this
      this.shooterRotationSubsystem.setAngle(angle);
      this.led.setLEDs();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterRotationSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.shooterRotationSubsystem.atSetpoint();
  }

  private boolean isInRange(double positionX) {
    return positionX <= Constants.Vision.MaxDistances.SPEAKER;
  }
}