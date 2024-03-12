package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_speedMotor;
    private final CANSparkMax m_rotationMotor;

    public Shooter() {
        this.m_speedMotor = new TalonFX(Constants.Shooter.SPEED_MOTOR_ID);
        Constants.Shooter.SPEED_MOTOR_CONTROL_LOOP_CONFIG.kS = 0;
        Constants.Shooter.SPEED_MOTOR_CONTROL_LOOP_CONFIG.kV = 0;
        Constants.Shooter.SPEED_MOTOR_CONTROL_LOOP_CONFIG.kP = 0;
        Constants.Shooter.SPEED_MOTOR_CONTROL_LOOP_CONFIG.kI = 0;
        Constants.Shooter.SPEED_MOTOR_CONTROL_LOOP_CONFIG.kD = 0;
        this.m_speedMotor.getConfigurator().apply(Constants.Shooter.SPEED_MOTOR_CONTROL_LOOP_CONFIG);

        this.m_rotationMotor = new CANSparkMax(Constants.Shooter.ROTATION_MOTOR_ID, MotorType.kBrushless);
    }

    public void setVelocitySetpoint(double velocity) { // RPM
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        this.m_speedMotor.setControl(request.withVelocity(velocity));
    }
}
