package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private final PIDCANSparkMax m_intakeMotor;
    private final CANSparkMax m_rotationMotor;

    private final DigitalInput m_topLimitSwitch;

    public Intake() {
        this.m_intakeMotor = new PIDCANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Intake.SPARK_PIDF_CONFIG);
        this.m_intakeMotor.getEncoder().setPositionConversionFactor(180); // idk if this will work

        this.m_rotationMotor = new PIDCANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless,
                IdleMode.kBrake, false, Constants.Intake.SPARK_PIDF_CONFIG);
        this.m_intakeMotor.getEncoder().setPositionConversionFactor(180); // idk if this will work

        this.m_topLimitSwitch = new DigitalInput(Constants.Intake.TOP_LIMIT_SWITCH_DIO_PORT);
    }

    public void setIntakeVelocitySetpoint(double velocity) {
        this.m_rotationMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public void setRotationAngleSetpoint(double angle) {
        this.m_rotationMotor.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void stopIntakeMotor() {
        this.m_intakeMotor.stopMotor();
    }

    public void stopRotationMotor() {
        this.m_rotationMotor.stopMotor();
    }

    public boolean isAtTopLimitSwitch() {
        return this.m_topLimitSwitch.get();
    }
}
