package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.uppermech.PIDCANSparkFlex;
import frc.robot.constants.Constants;

public class ShooterPower extends SubsystemBase {
    private final PIDCANSparkFlex motor1;
    private final PIDCANSparkFlex motor2;

    private final ConfiguredSimpleMotorFeedforward motor1FeedForward;
    private final ConfiguredSimpleMotorFeedforward motor2FeedForward;

    private boolean speedMotorsRunning = false;

    public ShooterPower() {
        // opposite direction spin to shoot
        this.motor1 = new PIDCANSparkFlex(Constants.Shooter.POWER_MOTOR_1_ID, MotorType.kBrushless,
                IdleMode.kCoast, false, Constants.Shooter.POWER_MOTOR_SPARK_PIDF_CONFIG);
        this.motor2 = new PIDCANSparkFlex(Constants.Shooter.POWER_MOTOR_2_ID, MotorType.kBrushless,
                IdleMode.kCoast, true, Constants.Shooter.POWER_MOTOR_SPARK_PIDF_CONFIG);

        this.motor1FeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Shooter.POWER_MOTOR_1_FF_CONFIG);
        this.motor2FeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Shooter.POWER_MOTOR_2_FF_CONFIG);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity",
                (this.motor1.getEncoder().getVelocity() +
                        this.motor2.getEncoder().getVelocity()) / 2.0);
        SmartDashboard.putBoolean("Shooter Running", this.speedMotorsRunning);
        // SmartDashboard.putNumber("Shooter Motor 1 Velocity",
        // this.m_speedMotor1.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Shooter Motor 2 Velocity",
        // this.m_speedMotor2.getEncoder().getVelocity());
    }

    public void setMaxVelocitySetpoint() {
        this.setMotorsVelocitySetpoint(Constants.Shooter.SPEED_MOTORS_MAX_RPM);
    }

    public void setMaxPercent() {
        this.motor1.set(0.725);
        this.motor2.set(0.725);

    }

    public void setMotorsVelocitySetpoint(double velocity) { // RPM
        this.motor1.getPIDController().setReference(this.motor1FeedForward.calculate(velocity),
                ControlType.kVoltage);
        this.motor2.getPIDController().setReference(this.motor2FeedForward.calculate(velocity),
                ControlType.kVoltage);
    }

    public void stopSpeedMotors() {
        this.speedMotorsRunning = false;
        this.motor1.stopMotor();
        this.motor2.stopMotor();
    }
}
