package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkFlex;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
    private final PIDCANSparkFlex m_speedMotor1;
    private final PIDCANSparkFlex m_speedMotor2;
    private final CANSparkMax m_rotationMotor;

    public Shooter() {
        this.m_speedMotor1 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_1_ID, MotorType.kBrushless,
                IdleMode.kBrake, false, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.m_speedMotor2 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_2_ID, MotorType.kBrushless,
                IdleMode.kBrake, false, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.m_rotationMotor = new CANSparkMax(Constants.Shooter.ROTATION_MOTOR_ID, MotorType.kBrushless);
    }

    public void setMaxVelocitySetpoint() {
        this.setVelocitySetpoint(Constants.Shooter.MAX_RPM / 1000); // lets NOT set max speed on the first try
    }

    public void setVelocitySetpoint(double velocity) { // RPM
        this.m_speedMotor1.getPIDController().setReference(velocity, ControlType.kVelocity);
        this.m_speedMotor2.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public void stopSpeedMotors() {
        this.m_speedMotor1.stopMotor();
        this.m_speedMotor2.stopMotor();
    }
}
