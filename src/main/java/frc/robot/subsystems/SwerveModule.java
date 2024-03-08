// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ProblemChildCANSparkMAX;
import frc.robot.Constants.Swerve;

public class SwerveModule extends SubsystemBase {
  private ProblemChildCANSparkMAX m_driveMotor;
  private ProblemChildCANSparkMAX m_angleMotor;

  private RelativeEncoder m_driveEncoder;
  private RelativeEncoder m_angleEncoder;

  private PIDController m_drivePIDController;

  private SimpleMotorFeedforward m_driveFeedforward;
  private SimpleMotorFeedforward m_angleFeedforward;

  private PIDController m_anglePIDController;
  private CANcoder m_angleAbsoluteEncoder;

  private boolean kAbsoluteEncoderReversed;
  private double kAbsoluteEncoderOffset;

  private Rotation2d m_lastAngle;

  final int steerId;
  final int driveId;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.kAbsoluteEncoderOffset = absoluteEncoderOffset;
    this.kAbsoluteEncoderReversed = absoluteEncoderReversed;

    this.m_angleAbsoluteEncoder = new CANcoder(absoluteEncoderId);

    this.m_driveMotor = new ProblemChildCANSparkMAX(driveMotorId, MotorType.kBrushless, IdleMode.kCoast,
        Swerve.DRIVE_MOTOR_SMART_LIMIT,
        driveMotorReversed);
    this.m_angleMotor = new ProblemChildCANSparkMAX(turnMotorId, MotorType.kBrushless, IdleMode.kCoast,
        Swerve.ANGLE_MOTOR_SMART_LIMIT,
        turnMotorReversed);

    this.m_driveEncoder = m_driveMotor.getEncoder();
    this.m_angleEncoder = m_angleMotor.getEncoder();

    this.m_drivePIDController = new PIDController(0, 0, 0);

    this.m_driveFeedforward = new SimpleMotorFeedforward(0.118, 2.617);
    this.m_angleFeedforward = new SimpleMotorFeedforward(0.132, 0);

    this.m_anglePIDController = new PIDController(Swerve.KP_TURNING, 0, 0);
    this.m_anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
    this.m_lastAngle = getState().angle;

    this.driveId = driveMotorId;
    this.steerId = turnMotorId;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(driveId + "DriveVelo", getDriveMotorVelocity());
    SmartDashboard.putNumber(steerId + "SteerPosition", getTurnMotorPosition() % 2 * Math.PI);
  }

  public void setBrake(boolean brake) {
    if (brake) {
      this.m_driveMotor.setIdleMode(IdleMode.kBrake);
      this.m_angleMotor.setIdleMode(IdleMode.kCoast);
    } else {
      this.m_driveMotor.setIdleMode(IdleMode.kCoast);
      this.m_angleMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getDriveMotorPosition() {
    return this.m_driveEncoder.getPosition() * Swerve.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity() {
    return this.m_driveEncoder.getVelocity() * Swerve.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition() {
    return this.m_angleEncoder.getPosition() * Swerve.ANGLE_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity() {
    return this.m_angleEncoder.getVelocity() * Swerve.ANGLE_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle() {
    double angle = this.m_angleAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= this.kAbsoluteEncoderOffset;
    angle *= (2 * Math.PI);
    return angle * (this.kAbsoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    this.m_driveEncoder.setPosition(0);
    this.m_angleEncoder.setPosition(getAbsoluteEncoderAngle() / Swerve.ANGLE_MOTOR_PCONVERSION);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getDriveMotorVelocity(), new Rotation2d(this.getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getDriveMotorPosition(), new Rotation2d(this.getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    this.setAngle(desiredState);
    this.setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + m_driveMotor.getDeviceId() + "] State", getState().toString());
  }

  public void setSpeed(SwerveModuleState desiredState) {
    double voltage = this.m_drivePIDController.calculate(this.getDriveMotorVelocity(),
        desiredState.speedMetersPerSecond);
    if (desiredState.speedMetersPerSecond == 0) {
      this.m_driveMotor.setVoltage(0);
      return;
    }
    voltage += this.m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(driveId + "DriveVoltage", voltage);
    this.m_driveMotor.setVoltage(voltage);
  }

  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.DRIVETRAIN_MAX_SPEED * 0.01))
        ? m_lastAngle
        : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    SmartDashboard.putNumber(steerId + "SteerSetpoint", desiredState.angle.getRadians());

    double voltage = this.m_anglePIDController.calculate(this.getTurnMotorPosition(), desiredState.angle.getRadians());
    voltage += this.m_angleFeedforward.calculate(0);
    this.m_angleMotor.setVoltage(voltage);
    this.m_lastAngle = angle;
  }

  public void stop() {
    m_driveMotor.set(0);
    m_angleMotor.set(0);
  }
}