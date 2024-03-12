package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkFlex;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex m_speedMotor1;
    private final CANSparkFlex m_speedMotor2;
    private final CANSparkMax m_rotationMotor;

    public Shooter() {
        this.m_speedMotor1 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_1_ID, MotorType.kBrushless,
                IdleMode.kBrake,
                40, false, 0, 1, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.m_speedMotor2 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_2_ID, MotorType.kBrushless,
                IdleMode.kBrake,
                40, false, 0, 1, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.m_rotationMotor = new CANSparkMax(Constants.Shooter.ROTATION_MOTOR_ID, MotorType.kBrushless);
    }

    public void setVelocitySetpoint(double velocity) { // RPM
        this.m_speedMotor1.getPIDController().setReference(velocity, ControlType.kVelocity);
        this.m_speedMotor2.getPIDController().setReference(velocity, ControlType.kVelocity);
    }
}
