package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.constants.Constants;

public class IntakePower extends SubsystemBase {
    private final PIDCANSparkMax motor;

    private final ConfiguredSimpleMotorFeedforward motorFeedForward;

    public IntakePower() {
        this.motor = new PIDCANSparkMax(Constants.Intake.POWER_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Intake.POWER_MOTOR_SPARK_PIDF_CONFIG);
        this.motorFeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Intake.POWER_MOTOR_FF_CONFIG);
    }

    @Override
    public void periodic() {
    }

    public void setMotorVelocitySetpoint(double velocity) {
        this.motor.getPIDController().setReference(this.motorFeedForward.calculate(velocity),
                ControlType.kVoltage);
    }

    public void stopMotor() {
        this.motor.stopMotor();
    }
}
