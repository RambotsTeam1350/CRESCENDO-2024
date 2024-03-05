// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ProblemChildCANSparkMAX;
import frc.robot.Constants.Swerve;

public class SwerveModule extends SubsystemBase {
  private final String kPosition;

  private ProblemChildCANSparkMAX m_driveMotor;
  private ProblemChildCANSparkMAX m_angleMotor;

  private RelativeEncoder m_driveEncoder;
  private RelativeEncoder m_angleEncoder;

  private PIDController m_turnPIDController;
  private CANcoder m_angleAbsoluteEncoder;

  private boolean kAbsoluteEncoderReversed;
  private double kAbsoluteEncoderOffset;

  private Rotation2d m_lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(String position, int driveMotorId, int turnMotorId, boolean driveMotorReversed,
      boolean turnMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.kPosition = position;
    this.kAbsoluteEncoderOffset = absoluteEncoderOffset;
    this.kAbsoluteEncoderReversed = absoluteEncoderReversed;

    this.m_angleAbsoluteEncoder = new CANcoder(absoluteEncoderId);

    this.m_driveMotor = new ProblemChildCANSparkMAX(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45,
        driveMotorReversed);
    this.m_angleMotor = new ProblemChildCANSparkMAX(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25,
        turnMotorReversed);

    this.m_driveEncoder = m_driveMotor.getEncoder();
    this.m_angleEncoder = m_angleMotor.getEncoder();

    this.m_turnPIDController = new PIDController(Swerve.KP_TURNING, Swerve.KI_TURNING, Swerve.KD_TURNING);
    this.m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.m_lastAngle = getState().angle;

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    SmartDashboard.putString("Swerve [" + kPosition + "] State", getState().toString());
  }

  public void setSpeed(SwerveModuleState desiredState) {
    this.m_driveMotor.set(desiredState.speedMetersPerSecond / Swerve.DRIVETRAIN_MAX_SPEED);
  }

  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.DRIVETRAIN_MAX_SPEED * 0.01))
        ? m_lastAngle
        : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    this.m_angleMotor
        .set(this.m_turnPIDController.calculate(this.getTurnMotorPosition(), desiredState.angle.getRadians()));
    this.m_lastAngle = angle;
  }

  public void stop() {
    m_driveMotor.set(0);
    m_angleMotor.set(0);
  }
}