package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    public Climber() {
        this.m_leftMotor = new PIDCANSparkMax(Constants.Climber.LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Climber.SPARK_PIDF_CONFIG);
        this.m_rightMotor = new PIDCANSparkMax(Constants.Climber.LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Climber.SPARK_PIDF_CONFIG);
    }

    public void setVoltageSetpoint(double voltage) {
        this.m_leftMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
        this.m_rightMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
    }
}
