package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private final PIDCANSparkMax powerMotor;
    private final CANSparkMax rotationMotor;

    private final DutyCycleEncoder throughBoreEncoder;
    private final DigitalInput topLimitSwitch;

    private final ConfiguredSimpleMotorFeedforward powerFeedForward;
    private final ConfiguredSimpleMotorFeedforward rotationFeedForward;

    public Intake() {
        this.powerMotor = new PIDCANSparkMax(Constants.Intake.POWER_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Intake.POWER_MOTOR_SPARK_PIDF_CONFIG);
        // this.m_intakeMotor.getEncoder().setPositionConversionFactor(Constants.Intake.ROTATION_MOTOR_PCONVERSION);
        // // idk
        // if
        // this
        // will
        // work

        this.rotationMotor = new PIDCANSparkMax(Constants.Intake.ROTATION_MOTOR_ID, MotorType.kBrushless,
                IdleMode.kBrake, false, Constants.Intake.ROTATION_MOTOR_SPARK_PIDF_CONFIG);
        // this.m_rotationMotor.getEncoder().setPositionConversionFactor(Constants.Intake.ROTATION_MOTOR_PCONVERSION);
        // // idk
        // if
        // this
        // will
        // work
        this.throughBoreEncoder = new DutyCycleEncoder(Constants.Intake.ROTATION_THROUGH_BORE_ENCODER_DIO_PORT);
        this.topLimitSwitch = new DigitalInput(Constants.Intake.TOP_LIMIT_SWITCH_DIO_PORT);

        this.rotationFeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Intake.ROTATION_MOTOR_FF_CONFIG);
        this.powerFeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Intake.POWER_MOTOR_FF_CONFIG);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake Relative Angle",
        // this.m_rotationMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Through Bore Angle", this.throughBoreEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("Intake Power Motor Velocity",
        // this.m_powerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake Rotation Speed", this.rotationMotor.getEncoder().getVelocity());
    }

    public void setPowerMotorVelocitySetpoint(double velocity) {
        this.powerMotor.getPIDController().setReference(this.powerFeedForward.calculate(velocity),
                ControlType.kVoltage);
    }

    public void setRotationMotorAngleSetpoint(double angle) {
        this.rotationMotor.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void setRotationMotorVelocitySetpoint(double velocity) {
        this.rotationMotor.getPIDController().setReference(this.rotationFeedForward.calculate(velocity),
                ControlType.kVoltage);
    }

    public void setRotationMotorVoltageSetpoint(double voltage) {
        this.rotationMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
    }

    public double getRotationAbsolutePosition() {
        return this.throughBoreEncoder.getAbsolutePosition();
    }

    public void stopPowerMotor() {
        this.powerMotor.stopMotor();
    }

    public void stopRotationMotor() {
        this.rotationMotor.stopMotor();
    }

    // public boolean isUpLimitSwitch() {
    // return !this.m_topLimitSwitch.get();
    // }
}
