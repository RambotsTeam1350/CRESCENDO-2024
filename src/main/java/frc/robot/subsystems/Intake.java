package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private final PIDCANSparkMax m_intakeMotor;
    private final CANSparkMax m_rotationMotor;

    private final DutyCycleEncoder m_throughBoreEncoder;
    private final DigitalInput m_topLimitSwitch;

    public Intake() {
        this.m_intakeMotor = new PIDCANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Intake.SPARK_PIDF_CONFIG);
        // this.m_intakeMotor.getEncoder().setPositionConversionFactor(Constants.Intake.ROTATION_MOTOR_PCONVERSION);
        // // idk
        // if
        // this
        // will
        // work

        this.m_rotationMotor = new PIDCANSparkMax(Constants.Intake.ROTATION_MOTOR_ID, MotorType.kBrushless,
                IdleMode.kBrake, false, Constants.Intake.SPARK_PIDF_CONFIG);
        // this.m_rotationMotor.getEncoder().setPositionConversionFactor(Constants.Intake.ROTATION_MOTOR_PCONVERSION);
        // // idk
        // if
        // this
        // will
        // work
        this.m_throughBoreEncoder = new DutyCycleEncoder(Constants.Intake.THROUGH_BORE_ENCODER_DIO_PORT);
        this.m_topLimitSwitch = new DigitalInput(Constants.Intake.TOP_LIMIT_SWITCH_DIO_PORT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Relative Angle", this.m_rotationMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Through Bore Angle", this.m_throughBoreEncoder.getAbsolutePosition());
        SmartDashboard.putBoolean("Intake Limit Switch", this.isUpLimitSwitch());
    }

    public void resetRotationMotorEncoder() {
        this.m_rotationMotor.getEncoder().setPosition(0);
    }

    public void setIntakeVelocitySetpoint(double velocity) {
        this.m_rotationMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public void setRotationAngleSetpoint(double angle) {
        this.m_rotationMotor.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void setRotationVoltageSetpoint(double voltage) {
        this.m_rotationMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
    }

    public double getRotationAbsoluteEncoder() {
        return this.m_throughBoreEncoder.getAbsolutePosition();
    }

    public void stopIntakeMotor() {
        this.m_intakeMotor.stopMotor();
    }

    public void stopRotationMotor() {
        this.m_rotationMotor.stopMotor();
    }

    public boolean isUpLimitSwitch() {
        return !this.m_topLimitSwitch.get();
    }
}
