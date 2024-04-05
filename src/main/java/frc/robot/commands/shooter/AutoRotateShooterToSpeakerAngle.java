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
import frc.robot.subsystems.LEDCANdle;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.vision.Camera;

public class AutoRotateShooterToSpeakerAngle extends Command {
  private final ShooterRotation shooterRotationSubsystem;
  private final Camera cameraSubsystem;
  private final LEDCANdle led;

  private boolean isInRange = false;

  public AutoRotateShooterToSpeakerAngle(ShooterRotation shooterRotation, Camera camera, LEDCANdle ledSubsystem) {
    this.shooterRotationSubsystem = shooterRotation;
    this.cameraSubsystem = camera;
    this.led = ledSubsystem;
    addRequirements(this.shooterRotationSubsystem);
    SmartDashboard.putData(this);
    // , this.camera, this.led); i am going to experiment with this
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ARSTSA STARTING");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("In Speaker Range", this.isInRange);
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
    SmartDashboard.putNumber("Distance From Speaker", shooterDistanceFromSpeaker);
    if (this.isInRange(shooterDistanceFromSpeaker)) {
      this.isInRange = true;
      double angle = Units.radiansToDegrees(
          Math.atan(
              Constants.Vision.Measurements.Speaker.SHOOTER_TO_GOAL_HEIGHT_METERS / shooterDistanceFromSpeaker));
      // angle -= Constants.Shooter.MAXIMUM_DEGREES_DOWN_ZERO_OFFSET;
      SmartDashboard.putNumber("Shooter Angle Setpoint", angle);
      // System.out.println("SHOOTER ANGLE SETPOINT: " + angle);
      this.shooterRotationSubsystem.setAngle(angle);
      // this.led.setLEDs();
    } else {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterRotationSubsystem.stopMotor();
    // this.led.stopLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.shooterRotationSubsystem.atSetpoint();
    // || this.cameraSubsystem.getSpeakerTarget() == null;
  }

  private boolean isInRange(double positionX) {
    return this.isInRange = positionX <= Constants.Vision.MaxDistances.SPEAKER;
  }
}