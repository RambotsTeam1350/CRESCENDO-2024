// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class Drivetrain extends SubsystemBase {
  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;

  private SlewRateLimiter m_frontLimiter;
  private SlewRateLimiter m_sideLimiter;
  private SlewRateLimiter m_turnLimiter;

  private Pigeon2 m_gyro;

  private SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    m_frontLeft = new SwerveModule(
        Swerve.FL.DRIVE_MOTOR_ID,
        Swerve.FL.ANGLE_MOTOR_ID,
        false,
        true,
        Swerve.FL.CANCODER_ID,
        Swerve.FL.CANCODER_ALIGNMENT_OFFSET,
        false);

    m_frontRight = new SwerveModule(
        Swerve.FR.DRIVE_MOTOR_ID,
        Swerve.FR.ANGLE_MOTOR_ID,
        false,
        true,
        Swerve.FR.CANCODER_ID,
        Swerve.FR.CANCODER_ALIGNMENT_OFFSET,
        false);

    m_backLeft = new SwerveModule(
        Swerve.BL.DRIVE_MOTOR_ID,
        Swerve.BL.ANGLE_MOTOR_ID,
        false,
        true,
        Swerve.BL.CANCODER_ID,
        Swerve.BL.CANCODER_ALIGNMENT_OFFSET,
        false);

    m_backRight = new SwerveModule(
        Swerve.BR.DRIVE_MOTOR_ID,
        Swerve.BR.ANGLE_MOTOR_ID,
        false,
        true,
        Swerve.BR.CANCODER_ID,
        Swerve.BR.CANCODER_ALIGNMENT_OFFSET,
        false);

    m_frontLimiter = new SlewRateLimiter(Swerve.TELE_DRIVE_MAX_ACCELERATION);
    m_sideLimiter = new SlewRateLimiter(Swerve.TELE_DRIVE_MAX_ACCELERATION);
    m_turnLimiter = new SlewRateLimiter(Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

    m_gyro = new Pigeon2(Swerve.PIGEON_ID);

    m_poseEstimator = new SwerveDrivePoseEstimator(
        Swerve.DRIVE_KINEMATICS,
        getHeadingRotation2d(),
        getModulePositions(),
        new Pose2d());

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
    m_poseEstimator.update(getHeadingRotation2d(), getModulePositions());

    SmartDashboard.putNumber("Robot Angle", getHeading());
    SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((-m_gyro.getRate() / 180)) + "pi rad/s");
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed,
      boolean fieldOriented, Translation2d centerOfRotation, boolean deadband) { // Drive with rotational speed control
                                                                                 // w/ joystick
    if (deadband) {
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;
    }

    frontSpeed = m_frontLimiter.calculate(frontSpeed) * Swerve.TELE_DRIVE_MAX_SPEED;
    sideSpeed = m_sideLimiter.calculate(sideSpeed) * Swerve.TELE_DRIVE_MAX_SPEED;
    turnSpeed = m_turnLimiter.calculate(turnSpeed) * Swerve.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void setAllIdleMode(boolean brake) {
    if (brake) {
      m_frontLeft.setBrake(true);
      m_frontRight.setBrake(true);
      m_backLeft.setBrake(true);
      m_backRight.setBrake(true);
    } else {
      m_frontLeft.setBrake(false);
      m_frontRight.setBrake(false);
      m_backLeft.setBrake(false);
      m_backRight.setBrake(false);
    }
  }

  public void resetAllEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Swerve.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  public void setHeading(double heading) {
    m_gyro.setYaw(heading);
  }

  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360); // clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_backLeft.stop();
    m_frontRight.stop();
    m_backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Swerve.DRIVETRAIN_MAX_SPEED);
    m_frontLeft.setDesiredState(moduleStates[0]);
    m_frontRight.setDesiredState(moduleStates[1]);
    m_backLeft.setDesiredState(moduleStates[2]);
    m_backRight.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_backLeft.getState();
    states[3] = m_backRight.getState();
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = m_frontLeft.getPosition();
    positions[1] = m_frontRight.getPosition();
    positions[2] = m_backLeft.getPosition();
    positions[3] = m_backRight.getPosition();
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