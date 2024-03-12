package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final PIDCANSparkMax m_intakeMotor;
    private final CANSparkMax m_rotationMotor;

    public Intake() {
        this.m_intakeMotor = new PIDCANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                40, false, 0, 1, Constants.Intake.SPARK_PIDF_CONFIG);
        this.m_rotationMotor = new CANSparkMax(Constants.Intake.ROTATION_MOTOR_ID, MotorType.kBrushless);
    }

    public void setVelocitySetpoint(double velocity) {
        this.m_intakeMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
    }
}
