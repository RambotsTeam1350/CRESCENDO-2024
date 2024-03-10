package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_speedMotor;
    private final TalonFX m_rotationMotor;

    public Shooter() {
        this.m_speedMotor = new CANSparkMax(Constants.Shooter.SPEED_MOTOR_ID, MotorType.kBrushless);
        this.m_rotationMotor = new TalonFX(Constants.Shooter.ROTATION_MOTOR_ID);
    }
}
