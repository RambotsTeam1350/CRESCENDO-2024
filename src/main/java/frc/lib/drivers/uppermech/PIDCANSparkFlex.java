package frc.lib.drivers.uppermech;

import com.revrobotics.CANSparkFlex;

import frc.lib.structs.PIDFConfig;

public class PIDCANSparkFlex extends CANSparkFlex {
    public PIDCANSparkFlex(int deviceId, MotorType m, IdleMode mode, boolean isInverted, double minOutput,
            double maxOutput,
            PIDFConfig pidfConfig) {
        super(deviceId, m);
        this.restoreFactoryDefaults();
        this.setIdleMode(mode);
        this.setInverted(isInverted);
        this.getPIDController().setOutputRange(minOutput, maxOutput);
        this.getPIDController().setP(pidfConfig.kP);
        this.getPIDController().setI(pidfConfig.kI);
        this.getPIDController().setD(pidfConfig.kD);
        this.getPIDController().setFF(pidfConfig.kF);
    }
}