package frc.lib.drivers.uppermech;

import com.revrobotics.CANSparkMax;

import frc.lib.structs.CANSparkPIDFConfig;

public class PIDCANSparkMax extends CANSparkMax {
    public PIDCANSparkMax(int deviceId, MotorType m, IdleMode mode, boolean isInverted,
            CANSparkPIDFConfig sparkPidfConfig) {
        super(deviceId, m);
        this.restoreFactoryDefaults();
        this.setIdleMode(mode);
        this.setInverted(isInverted);
        this.getPIDController().setOutputRange(sparkPidfConfig.kMinOutput, sparkPidfConfig.kMaxOutput);
        this.getPIDController().setP(sparkPidfConfig.kP);
        this.getPIDController().setI(sparkPidfConfig.kI);
        this.getPIDController().setD(sparkPidfConfig.kD);
        this.getPIDController().setFF(sparkPidfConfig.kFF);
    }
}
