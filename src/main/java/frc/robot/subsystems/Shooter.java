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
    private final PIDCANSparkFlex speedMotor1;
    private final PIDCANSparkFlex speedMotor2;
    private final CANSparkMax rotationMotor;

    private final ConfiguredSimpleMotorFeedforward speedMotor1FeedForward;
    private final ConfiguredSimpleMotorFeedforward speedMotor2FeedForward;

    private boolean speedMotorsRunning = false;

    public Shooter() {
        // opposite direction spin to shoot
        this.speedMotor1 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_1_ID, MotorType.kBrushless,
                IdleMode.kCoast, false, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.speedMotor2 = new PIDCANSparkFlex(Constants.Shooter.SPEED_MOTOR_2_ID, MotorType.kBrushless,
                IdleMode.kCoast, true, Constants.Shooter.SPARK_PIDF_CONFIG);
        this.rotationMotor = new CANSparkMax(Constants.Shooter.ROTATION_MOTOR_ID, MotorType.kBrushless);

        this.speedMotor1FeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Shooter.SPEED_MOTOR_1_FF_CONFIG);
        this.speedMotor2FeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Shooter.SPEED_MOTOR_2_FF_CONFIG);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity",
                (this.speedMotor1.getEncoder().getVelocity() +
                        this.speedMotor2.getEncoder().getVelocity()) / 2.0);
        SmartDashboard.putBoolean("Shooter Running", this.speedMotorsRunning);
        // SmartDashboard.putNumber("Shooter Motor 1 Velocity",
        // this.m_speedMotor1.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Shooter Motor 2 Velocity",
        // this.m_speedMotor2.getEncoder().getVelocity());
    }

    public void setMaxVelocitySetpoint() {
        this.setSpeedMotorsVelocitySetpoint(Constants.Shooter.SPEED_MOTORS_MAX_RPM);
    }

    public void setSpeedMotorsVelocitySetpoint(double velocity) { // RPM
        this.speedMotor1.getPIDController().setReference(this.speedMotor1FeedForward.calculate(velocity),
                ControlType.kVoltage);
        this.speedMotor2.getPIDController().setReference(this.speedMotor2FeedForward.calculate(velocity),
                ControlType.kVoltage);
    }

    public void stopSpeedMotors() {
        this.speedMotorsRunning = false;
        this.speedMotor1.stopMotor();
        this.speedMotor2.stopMotor();
    }

    public void stopRotationMotor() {
        this.rotationMotor.stopMotor();
    }
}
