// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredPIDController;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.swerve.SwerveCANSparkMAX;
import frc.robot.constants.Constants.Swerve;

public class SwerveModule extends SubsystemBase {
  private SwerveCANSparkMAX driveMotor;
  private SwerveCANSparkMAX angleMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder angleEncoder;

  private ConfiguredPIDController drivePIDController;
  private ConfiguredPIDController anglePIDController;

  private ConfiguredSimpleMotorFeedforward driveFeedforward;
  private ConfiguredSimpleMotorFeedforward angleFeedforward;

  private CANcoder angleAbsoluteEncoder;

  private boolean kAbsoluteEncoderReversed;
  private double kAbsoluteEncoderOffset;

  private Rotation2d lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.kAbsoluteEncoderOffset = absoluteEncoderOffset;
    this.kAbsoluteEncoderReversed = absoluteEncoderReversed;

    this.angleAbsoluteEncoder = new CANcoder(absoluteEncoderId);

    this.driveMotor = new SwerveCANSparkMAX(driveMotorId, MotorType.kBrushless, IdleMode.kCoast,
        Swerve.DRIVE_MOTOR_SMART_LIMIT,
        driveMotorReversed);
    this.angleMotor = new SwerveCANSparkMAX(turnMotorId, MotorType.kBrushless, IdleMode.kCoast,
        Swerve.ANGLE_MOTOR_SMART_LIMIT,
        turnMotorReversed);

    this.driveEncoder = driveMotor.getEncoder();
    this.angleEncoder = angleMotor.getEncoder();

    this.drivePIDController = new ConfiguredPIDController(Swerve.DRIVE_MOTOR_PID_CONFIG);
    this.anglePIDController = new ConfiguredPIDController(Swerve.ANGLE_MOTOR_PID_CONFIG);
    this.anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveFeedforward = new ConfiguredSimpleMotorFeedforward(Swerve.DRIVE_MOTOR_FF_CONFIG);
    this.angleFeedforward = new ConfiguredSimpleMotorFeedforward(Swerve.ANGLE_MOTOR_FF_CONFIG);

    resetEncoders();
    this.lastAngle = getState().angle;
  }

  public void setBrake(boolean brake) {
    if (brake) {
      this.driveMotor.setIdleMode(IdleMode.kBrake);
      this.angleMotor.setIdleMode(IdleMode.kCoast);
    } else {
      this.driveMotor.setIdleMode(IdleMode.kCoast);
      this.angleMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getDriveMotorPosition() {
    return this.driveEncoder.getPosition() * Swerve.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity() {
    return this.driveEncoder.getVelocity() * Swerve.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition() {
    return this.angleEncoder.getPosition() * Swerve.ANGLE_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity() {
    return this.angleEncoder.getVelocity() * Swerve.ANGLE_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle() {
    double angle = this.angleAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= this.kAbsoluteEncoderOffset;
    angle *= (2 * Math.PI);
    return angle * (this.kAbsoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    this.driveEncoder.setPosition(0);
    this.angleEncoder.setPosition(getAbsoluteEncoderAngle() / Swerve.ANGLE_MOTOR_PCONVERSION);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getDriveMotorVelocity(), new Rotation2d(this.getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getDriveMotorPosition(), new Rotation2d(this.getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = SwerveModuleState.optimize(desiredState, this.getState().angle);

    this.setAngle(desiredState);
    this.setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + this.driveMotor.getDeviceId() + "] State", getState().toString());
  }

  public void setSpeed(SwerveModuleState desiredState) {
    if (desiredState.speedMetersPerSecond == 0) {
      this.driveMotor.setVoltage(0);
      return;
    }
    double voltage = this.drivePIDController.calculate(this.getDriveMotorVelocity(),
        desiredState.speedMetersPerSecond);
    voltage += this.driveFeedforward.calculate(desiredState.speedMetersPerSecond);
    this.driveMotor.setVoltage(voltage);
  }

  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.DRIVETRAIN_MAX_SPEED * 0.01))
        ? lastAngle
        : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    double voltage = this.anglePIDController.calculate(this.getTurnMotorPosition(), desiredState.angle.getRadians());
    voltage += this.angleFeedforward.calculate(0) * Math.signum(voltage);
    this.angleMotor.setVoltage(voltage);
    this.lastAngle = angle;
  }

  public void stop() {
    this.driveMotor.set(0);
    this.angleMotor.set(0);
  }
}