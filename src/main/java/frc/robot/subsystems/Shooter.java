package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_speedMotor;
    private final CANSparkMax m_rotationMotor;

    public Shooter() {
        this.m_speedMotor = new PIDCANSparkMax(Constants.Shooter.SPEED_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                40, false, 0, 1, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.m_rotationMotor = new CANSparkMax(Constants.Shooter.ROTATION_MOTOR_ID, MotorType.kBrushless);
    }

    public void setVelocitySetpoint(double velocity) { // RPM
        this.m_speedMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
    }
}
