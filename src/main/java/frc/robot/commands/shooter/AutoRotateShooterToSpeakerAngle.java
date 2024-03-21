// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.intake.IntakeRotation;
import frc.robot.subsystems.vision.Camera;

public class AutoRotateShooterToSpeakerAngle extends Command {
  private final IntakeRotation intakeRotation;
  private final Camera camera;
  private final LED led;

  private final double kTargetHeightMeters = Units.inchesToMeters(105);

  private double angle;

  public AutoRotateShooterToSpeakerAngle(IntakeRotation intakeRotation, Camera camera, LED led) {
    this.intakeRotation = intakeRotation;
    this.camera = camera;
    this.led = led;
    addRequirements(this.intakeRotation, this.camera, this.led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double positionX = PhotonUtils.calculateDistanceToTargetMeters(
        Constants.Vision.CAMERA_HEIGHT_METERS,
        this.kTargetHeightMeters,
        Constants.Vision.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(this.camera.getBestTarget().getPitch()));

    if (this.camera.hasTarget() && this.isInRange(positionX)) {
      this.angle = Math.atan((1.5) / positionX);
      this.intakeRotation.setAngle(angle);
      this.led.setLEDs();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isInRange(double positionX) {
    return (this.camera.getTargetFiducialID() == Constants.Vision.FiducialIDs.SPEAKER)
        && (positionX <= Constants.Shooter.VISION_MAX_DISTANCE);
  }
}
