package frc.robot.subsystems.intake;

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

public class IntakeRotation extends SubsystemBase {
    private final CANSparkMax motor;

    private final DutyCycleEncoder throughBoreEncoder;
    private final DigitalInput topLimitSwitch;

    private final ConfiguredSimpleMotorFeedforward motorFeedForward;

    public IntakeRotation() {
        this.motor = new PIDCANSparkMax(Constants.Intake.ROTATION_MOTOR_ID, MotorType.kBrushless,
                IdleMode.kBrake, false, Constants.Intake.ROTATION_MOTOR_SPARK_PIDF_CONFIG);

        this.throughBoreEncoder = new DutyCycleEncoder(Constants.Intake.ROTATION_THROUGH_BORE_ENCODER_DIO_PORT);

        this.topLimitSwitch = new DigitalInput(Constants.Intake.TOP_LIMIT_SWITCH_DIO_PORT);

        this.motorFeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Intake.ROTATION_MOTOR_FF_CONFIG);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake Relative Angle",
        // this.rotationMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Through Bore Angle", this.throughBoreEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Intake Rotation Speed", this.motor.getEncoder().getVelocity());
    }

    public void setMotorVelocitySetpoint(double velocity) {
        this.motor.getPIDController().setReference(this.motorFeedForward.calculate(velocity),
                ControlType.kVoltage);
    }

    // public void setMotorVoltageSetpoint(double voltage) {
    // this.rotationMotor.getPIDController().setReference(voltage,
    // ControlType.kVoltage);
    // }

    public double getEncoderAbsolutePosition() {
        return this.throughBoreEncoder.getAbsolutePosition();
    }

    public void stopMotor() {
        this.motor.stopMotor();
    }

    // public boolean isUpLimitSwitch() {
    // return !this.m_topLimitSwitch.get();
    // }
}
