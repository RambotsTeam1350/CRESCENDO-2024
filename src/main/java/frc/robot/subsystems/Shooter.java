package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.uppermech.PIDCANSparkFlex;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
    private final PIDCANSparkFlex m_speedMotor1;
    private final PIDCANSparkFlex m_speedMotor2;
    private final CANSparkMax m_rotationMotor;

    private final ConfiguredSimpleMotorFeedforward m_speedMotor1FeedForward;
    private final ConfiguredSimpleMotorFeedforward m_speedMotor2FeedForward;

    public Shooter() {
        // opposite direction spin to shoot
        this.m_speedMotor1 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_1_ID, MotorType.kBrushless,
                IdleMode.kCoast, false, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.m_speedMotor2 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_2_ID, MotorType.kBrushless,
                IdleMode.kCoast, true, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.m_rotationMotor = new CANSparkMax(Constants.Shooter.ROTATION_MOTOR_ID, MotorType.kBrushless);

        this.m_speedMotor1FeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Shooter.SPEED_MOTOR_1_FF_CONFIG);
        this.m_speedMotor2FeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Shooter.SPEED_MOTOR_2_FF_CONFIG);
    }

    // FOR TESTING PURPOSE
    public void setVoltage(double voltage) {
        this.m_speedMotor1.setVoltage(voltage);
        this.m_speedMotor2.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shooter Velocity",
        // (this.m_speedMotor1.getEncoder().getVelocity() +
        // this.m_speedMotor2.getEncoder().getVelocity()) / 2.0);
        SmartDashboard.putNumber("Shooter Motor 1 Velocity", this.m_speedMotor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Motor 2 Velocity", this.m_speedMotor2.getEncoder().getVelocity());
    }

    public void setMaxVelocitySetpoint() {
        this.setVelocitySetpoint(Constants.Shooter.MAX_RPM);
    }

    public void setVelocity(double velocity) {
        this.m_speedMotor1.set(this.m_speedMotor1FeedForward.calculate(velocity));
        this.m_speedMotor2.set(this.m_speedMotor2FeedForward.calculate(velocity));
    }

    public void setVelocitySetpoint(double velocity) { // RPM
        this.m_speedMotor1.getPIDController().setReference(velocity, ControlType.kVelocity);
        this.m_speedMotor2.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public void setVelocitySetpointFF(double velocity) { // RPM
        this.m_speedMotor1.getPIDController().setReference(this.m_speedMotor1FeedForward.calculate(velocity),
                ControlType.kVoltage);
        this.m_speedMotor2.getPIDController().setReference(this.m_speedMotor1FeedForward.calculate(velocity),
                ControlType.kVoltage);
    }

    public void stopSpeedMotors() {
        this.m_speedMotor1.stopMotor();
        this.m_speedMotor2.stopMotor();
    }

    public void stopRotationMotor() {
        this.m_rotationMotor.stopMotor();
    }
}
