package frc.lib.drivers.uppermech;

import com.revrobotics.CANSparkMax;

import frc.lib.structs.PIDFConfig;

public class PIDCANSparkMax extends CANSparkMax {
    public PIDCANSparkMax(int deviceId, MotorType m, IdleMode mode, int limit, boolean isInverted, double minOutput,
            double maxOutput,
            PIDFConfig pidfConfig) {
        super(deviceId, m);
        this.restoreFactoryDefaults();
        this.setIdleMode(mode);
        this.setSmartCurrentLimit(limit);
        this.setInverted(isInverted);
        this.getPIDController().setOutputRange(minOutput, maxOutput);
        this.getPIDController().setP(pidfConfig.kP);
        this.getPIDController().setI(pidfConfig.kI);
        this.getPIDController().setD(pidfConfig.kD);
        this.getPIDController().setFF(pidfConfig.kF);
    }
}
