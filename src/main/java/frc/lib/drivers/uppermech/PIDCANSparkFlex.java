package frc.lib.drivers.uppermech;

import com.revrobotics.CANSparkFlex;

import frc.lib.structs.CANSparkPIDFConfig;

/**
 * Creates a new {@link CANSparkMax} with the necessary configurations.
 * 
 * @param deviceId   The device ID.
 * @param motorType  The motor type (Brushed/Brushless).
 * @param mode       The idle mode (kBrake/kCoast).
 * @param isInverted The invert type of the motor.
 * @param minOutput  The minimum PID output.
 * @param maxOutput  The maximum PID output.
 * @param pidfConfig The {@link CANSparkPIDFConfig} constants to configure the
 *                   onboard
 *                   PIDF controller with.
 */

public class PIDCANSparkFlex extends CANSparkFlex {
    public PIDCANSparkFlex(int deviceId, MotorType motorType, IdleMode idleMode, int currentLimit, boolean isInverted,
            CANSparkPIDFConfig pidfConfig) {
        super(deviceId, motorType);
        this.restoreFactoryDefaults();
        this.setIdleMode(idleMode);
        this.setSmartCurrentLimit(currentLimit);
        this.setInverted(isInverted);
        this.getPIDController().setOutputRange(pidfConfig.kMinOutput, pidfConfig.kMaxOutput);
        this.getPIDController().setP(pidfConfig.kP);
        this.getPIDController().setI(pidfConfig.kI);
        this.getPIDController().setD(pidfConfig.kD);
        this.getPIDController().setFF(pidfConfig.kFF);
    }
}
