// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.vision.Camera;

public class AutoRotateShooterToSpeakerAngle extends Command {
  private final ShooterRotation shooterRotation;
  private final Camera camera;
  private final LED led;

  private final double kTargetHeightMeters = Units.inchesToMeters(105);

  private double angle;

  public AutoRotateShooterToSpeakerAngle(ShooterRotation shooterRotation, Camera camera, LED led) {
    this.shooterRotation = shooterRotation;
    this.camera = camera;
    this.led = led;
    addRequirements(this.shooterRotation, this.camera, this.led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SHOOTER COMMAND");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double positionX = PhotonUtils.calculateDistanceToTargetMeters(
        Constants.Vision.CAMERA_HEIGHT_METERS,
        this.kTargetHeightMeters,
        Constants.Vision.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(this.camera.getBestTarget().getPitch()));
    System.out.println("POSITION X: " + positionX);

    if (this.camera.hasTarget() && this.isInRange(positionX)) {
      this.angle = Math.atan((Units.radiansToDegrees(1.5)) / positionX);
      System.out.println("SHOOTER ANGLE SETPOINT: " + angle);
      // angle = 0.0;
      // this.shooterRotation.setAngle(angle);
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
    return (this.camera.getTargetFiducialID() == (Constants.Vision.FiducialIDs.SPEAKER_RED)
        || this.camera.getTargetFiducialID() == Constants.Vision.FiducialIDs.SPEAKER_BLUE)
        && (positionX <= Constants.Vision.MaxDistances.SHOOTER);
  }
}
