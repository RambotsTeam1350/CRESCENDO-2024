package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.ConfiguredPIDController;
import frc.lib.drivers.ConfiguredSimpleMotorFeedforward;
import frc.lib.drivers.uppermech.ConfiguredCANSparkMax;
import frc.robot.constants.Constants;

public class IntakeRotation extends SubsystemBase {
    private final CANSparkMax motor;

    private final DutyCycleEncoder throughBoreEncoder;

    private final DigitalInput topLimitSwitch;

    private final ConfiguredPIDController PIDController;
    private final ConfiguredSimpleMotorFeedforward motorFeedForward;

    public IntakeRotation() {
        this.motor = new ConfiguredCANSparkMax(Constants.Intake.ROTATION_MOTOR_ID, MotorType.kBrushless,
                IdleMode.kBrake, 40, true);

        this.throughBoreEncoder = new DutyCycleEncoder(Constants.Intake.ROTATION_THROUGH_BORE_ENCODER_DIO_PORT);
        this.throughBoreEncoder.setPositionOffset(Constants.Intake.ROTATION_THROUGH_BORE_ENCODER_POSITION_OFFSET);

        this.topLimitSwitch = new DigitalInput(Constants.Intake.TOP_LIMIT_SWITCH_DIO_PORT);

        this.PIDController = new ConfiguredPIDController(Constants.Intake.ROTATION_MOTOR_PID_CONFIG);
        this.PIDController.setTolerance(2);
        this.motorFeedForward = new ConfiguredSimpleMotorFeedforward(Constants.Intake.ROTATION_MOTOR_FF_CONFIG);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Through Bore Angle", this.getAngle());
    }

    /**
     * Sets the angle of the intake using PID closed loop control
     * 
     * @param angle the desired angle in degrees
     */
    public void setAngle(double angle) {
        // to prevent Accidents(TM)
        angle = MathUtil.clamp(angle, Constants.Intake.MAXIMUM_DEGREES_UP, Constants.Intake.MAXIMUM_DEGREES_DOWN);
        double voltage = this.PIDController.calculate(this.getAngle(), angle);
        // 0 velocity because we do not care about the velocity of the rotation motor,
        // we just want to get it to a specific angle
        voltage += (this.motorFeedForward.calculate(0) * Math.signum(voltage));
        this.motor.setVoltage(voltage);
    }

    /**
     * 
     * @return the angle of the intake in degrees
     */
    public double getAngle() {
        // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html#getAbsolutePosition()
        return (this.throughBoreEncoder.getAbsolutePosition() - this.throughBoreEncoder.getPositionOffset()) * 360.0;
    }

    public void stopMotor() {
        this.motor.stopMotor();
    }

    /**
     * 
     * @return whether the intake rotation is within 5 degrees of the desired angle
     */
    public boolean atSetpoint() {
        return this.PIDController.atSetpoint();
    }

    // public boolean isUpLimitSwitch() {
    // return !this.m_topLimitSwitch.get();
    // }
}
