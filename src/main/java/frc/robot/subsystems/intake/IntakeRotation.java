package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredPIDController;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.constants.Constants;

public class IntakeRotation extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final DutyCycleEncoder throughBoreEncoder;
    private final DigitalInput topLimitSwitch;

    private final ConfiguredPIDController PIDController;
    private final ConfiguredSimpleMotorFeedforward motorFeedForward;

    public IntakeRotation() {
        this.motor = new PIDCANSparkMax(Constants.Intake.ROTATION_MOTOR_ID, MotorType.kBrushless,
                IdleMode.kBrake, true, Constants.Intake.ROTATION_MOTOR_SPARK_PIDF_CONFIG);

        this.encoder = this.motor.getEncoder();
        // this.encoder.setPositionConversionFactor(360);

        this.throughBoreEncoder = new DutyCycleEncoder(Constants.Intake.ROTATION_THROUGH_BORE_ENCODER_DIO_PORT);
        this.throughBoreEncoder.setPositionOffset(0.320);

        this.topLimitSwitch = new DigitalInput(Constants.Intake.TOP_LIMIT_SWITCH_DIO_PORT);

        this.PIDController = new ConfiguredPIDController(Constants.Intake.ROTATION_MOTOR_PID_CONFIG);
        this.PIDController.setTolerance(5);

        this.motorFeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Intake.ROTATION_MOTOR_FF_CONFIG);

        // this.encoder.setPosition(this.throughBoreEncoder.getAbsolutePosition() *
        // 360.0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake Relative Angle",
        // this.encoder.getPosition() *
        // Constants.Intake.ROTATION_POSITION_CONVERSION_FACTOR);
        // SmartDashboard.putNumber("Intake Through Bore Angle",
        // this.throughBoreEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Intake Through Bore Angle",
                (this.throughBoreEncoder.getAbsolutePosition() - this.throughBoreEncoder.getPositionOffset()) * 360.0);
        SmartDashboard.putNumber("raw abs pos", this.throughBoreEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Intake Rotation Speed", this.motor.getEncoder().getVelocity());
    }

    public void setAngle(double angle) {
        double voltage = this.PIDController.calculate(this.getAngle(), angle);
        voltage += this.motorFeedForward.calculate(0);
        System.out.println(voltage);
        this.motor.setVoltage(voltage);
    }

    public void setMotorPositionSetpoint(double position) {
        this.motor.getPIDController().setReference(position, ControlType.kPosition);
    }

    // public void setMotorVoltageSetpoint(double voltage) {
    // this.rotationMotor.getPIDController().setReference(voltage,
    // ControlType.kVoltage);
    // }

    public double getAngle() {
        return (this.throughBoreEncoder.getAbsolutePosition() - this.throughBoreEncoder.getPositionOffset()) * 360.0;
    }

    public void stopMotor() {
        this.motor.stopMotor();
    }

    public boolean atSetpoint() {
        return this.PIDController.atSetpoint();
    }

    // public boolean isUpLimitSwitch() {
    // return !this.m_topLimitSwitch.get();
    // }
}
