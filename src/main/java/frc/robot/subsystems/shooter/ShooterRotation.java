package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredPIDController;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.uppermech.ConfiguredCANSparkMax;
import frc.robot.constants.Constants;

public class ShooterRotation extends SubsystemBase {
    private final CANSparkMax motor;

    // private final DutyCycleEncoder throughBoreEncoder;

    private final ConfiguredPIDController PIDController;
    private final ConfiguredSimpleMotorFeedforward motorFeedForward;

    public ShooterRotation() {
        this.motor = new ConfiguredCANSparkMax(Constants.Shooter.ROTATION_MOTOR_ID, MotorType.kBrushless,
                IdleMode.kBrake, false);

        // this.throughBoreEncoder = new
        // DutyCycleEncoder(Constants.Shooter.ROTATION_THROUGH_BORE_ENCODER_DIO_PORT);
        // this.throughBoreEncoder.setPositionOffset(Constants.Shooter.ROTATION_THROUGH_BORE_ENCODER_POSITION_OFFSET);

        this.PIDController = new ConfiguredPIDController(Constants.Shooter.ROTATION_MOTOR_PID_CONFIG);
        this.PIDController.setTolerance(2); // TODO: figure out tolerance

        this.motorFeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Shooter.ROTATION_MOTOR_FF_CONFIG);
    }

    @Override
    public void periodic() {
    }

    /**
     * Sets the angle of the intake using PID closed loop control
     * 
     * @param angle the desired angle in degrees
     */
    // public void setAngle(double angle) {
    // double voltage = this.PIDController.calculate(this.getAngle(), angle);
    // // 0 velocity because we do not care about the velocity of the rotation
    // motor,
    // // we just want to get it to a specific angle
    // voltage += this.motorFeedForward.calculate(0);
    // System.out.println(voltage);
    // this.motor.setVoltage(voltage);
    // }

    /**
     * 
     * @return the angle of the intake in degrees
     */
    // public double getAngle() {
    // //
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html#getAbsolutePosition()
    // return (this.throughBoreEncoder.getAbsolutePosition() -
    // this.throughBoreEncoder.getPositionOffset()) * 360.0;
    // }

    public void stopMotor() {
        this.motor.stopMotor();
    }
}