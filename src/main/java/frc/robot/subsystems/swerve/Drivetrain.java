// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.text.DecimalFormat;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredPIDController;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.vision.Camera;

public class Drivetrain extends SubsystemBase {
  private final Camera cameraSubsystem;

  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  private SlewRateLimiter frontLimiter;
  private SlewRateLimiter sideLimiter;
  private SlewRateLimiter turnLimiter;

  private Pigeon2 gyro;
  private ConfiguredPIDController headingPIDController = new ConfiguredPIDController(Swerve.HEADING_PID_CONFIG);

  private SwerveDrivePoseEstimator poseEstimator;

  private Field2d field;

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain(Camera cameraSubsystem) {
    this.cameraSubsystem = cameraSubsystem;

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    this.frontLeft = new SwerveModule(
        Swerve.FL.DRIVE_MOTOR_ID,
        Swerve.FL.ANGLE_MOTOR_ID,
        true,
        true,
        Swerve.FL.CANCODER_ID,
        Swerve.FL.CANCODER_ALIGNMENT_OFFSET,
        false);

    this.frontRight = new SwerveModule(
        Swerve.FR.DRIVE_MOTOR_ID,
        Swerve.FR.ANGLE_MOTOR_ID,
        true,
        true,
        Swerve.FR.CANCODER_ID,
        Swerve.FR.CANCODER_ALIGNMENT_OFFSET,
        false);

    this.backLeft = new SwerveModule(
        Swerve.BL.DRIVE_MOTOR_ID,
        Swerve.BL.ANGLE_MOTOR_ID,
        true,
        true,
        Swerve.BL.CANCODER_ID,
        Swerve.BL.CANCODER_ALIGNMENT_OFFSET,
        false);

    this.backRight = new SwerveModule(
        Swerve.BR.DRIVE_MOTOR_ID,
        Swerve.BR.ANGLE_MOTOR_ID,
        true,
        true,
        Swerve.BR.CANCODER_ID,
        Swerve.BR.CANCODER_ALIGNMENT_OFFSET,
        false);

    this.frontLimiter = new SlewRateLimiter(Swerve.TELE_DRIVE_MAX_ACCELERATION);
    this.sideLimiter = new SlewRateLimiter(Swerve.TELE_DRIVE_MAX_ACCELERATION);
    this.turnLimiter = new SlewRateLimiter(Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

    this.gyro = new Pigeon2(Swerve.PIGEON_ID);

    this.poseEstimator = new SwerveDrivePoseEstimator(
        Swerve.DRIVE_KINEMATICS,
        getHeadingRotation2d(),
        getModulePositions(),
        new Pose2d());

    this.field = new Field2d();
    SmartDashboard.putData(this.field);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        Swerve.AUTO_CONFIG,
        () -> isRedAlliance(),
        this);
  }

  @Override
  public void periodic() {
    this.poseEstimator.update(getHeadingRotation2d(), getModulePositions());
    // Optional<EstimatedRobotPose> visionEstimatedRobotPose =
    // this.cameraSubsystem.getEstimatedRobotPose();
    // if (visionEstimatedRobotPose.isPresent()) {
    // this.poseEstimator.addVisionMeasurement(visionEstimatedRobotPose.get().estimatedPose.toPose2d(),
    // visionEstimatedRobotPose.get().timestampSeconds);
    // }
    this.field.setRobotPose(getPose());

    SmartDashboard.putNumber("Robot Angle", getHeading());
    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "pi rad/s");
  }

  public void controllerDrive(double frontSpeed, double sideSpeed, double turnSpeed,
      boolean fieldOriented, boolean halfSpeed, Translation2d centerOfRotation, boolean deadband) { // Drive with
                                                                                                    // rotational speed
                                                                                                    // control
    // w/ joystick
    if (halfSpeed) {
      frontSpeed *= 0.25;
      sideSpeed *= 0.25;
      turnSpeed *= 0.25;
    }
    if (deadband) {
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * Swerve.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * Swerve.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * Swerve.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    this.setModuleStates(moduleStates);
  }

  public void rotateToAngle(double angle) { // UNTESTED BE CAREFUL
    double turnSpeed = -this.headingPIDController.calculate(this.getHeading(), angle);
    turnSpeed = turnLimiter.calculate(turnSpeed) * Swerve.TELE_DRIVE_MAX_ANGULAR_SPEED;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turnSpeed);
    SwerveModuleState[] moduleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);
  }

  // see AutoAlignToSpeaker
  public void rotateToFaceVisionTarget(double currentYaw) {
    double turnSpeed = this.headingPIDController.calculate(currentYaw, 0);
    turnSpeed = turnLimiter.calculate(turnSpeed) * Swerve.TELE_DRIVE_MAX_ANGULAR_SPEED;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turnSpeed);
    SwerveModuleState[] moduleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);
  }

  public boolean isAtHeadingSetpoint() {
    return this.headingPIDController.atSetpoint();
  }

  public void setAllIdleMode(boolean brake) {
    if (brake) {
      this.frontLeft.setBrake(true);
      this.frontRight.setBrake(true);
      this.backLeft.setBrake(true);
      this.backRight.setBrake(true);
    } else {
      this.frontLeft.setBrake(false);
      this.frontRight.setBrake(false);
      this.backLeft.setBrake(false);
      this.backRight.setBrake(false);
    }
  }

  public void resetAllEncoders() {
    this.frontLeft.resetEncoders();
    this.frontRight.resetEncoders();
    this.backLeft.resetEncoders();
    this.backRight.resetEncoders();
  }

  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    this.poseEstimator.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Swerve.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] moduleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);
  }

  public void zeroHeading() {
    this.gyro.setYaw(0);
  }

  public void setHeading(double heading) {
    this.gyro.setYaw(heading);
  }

  public double getHeading() {
    return Math.IEEEremainder(-this.gyro.getAngle(), 360); // clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(this.getHeading());
  }

  public void stopModules() {
    this.frontLeft.stop();
    this.backLeft.stop();
    this.frontRight.stop();
    this.backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Swerve.DRIVETRAIN_MAX_SPEED);
    this.frontLeft.setDesiredState(moduleStates[0]);
    this.frontRight.setDesiredState(moduleStates[1]);
    this.backLeft.setDesiredState(moduleStates[2]);
    this.backRight.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = this.frontLeft.getState();
    states[1] = this.frontRight.getState();
    states[2] = this.backLeft.getState();
    states[3] = this.backRight.getState();
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = this.frontLeft.getPosition();
    positions[1] = this.frontRight.getPosition();
    positions[2] = this.backLeft.getPosition();
    positions[3] = this.backRight.getPosition();
    return positions;
  }

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}