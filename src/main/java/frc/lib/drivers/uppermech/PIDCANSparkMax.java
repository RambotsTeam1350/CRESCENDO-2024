package frc.lib.drivers.uppermech;

import com.revrobotics.CANSparkMax;

import frc.lib.structs.CANSparkPIDFConfig;

public class PIDCANSparkMax extends CANSparkMax {
    public PIDCANSparkMax(int deviceId, MotorType motorType, IdleMode idleMode, boolean isInverted,
            CANSparkPIDFConfig sparkPidfConfig) {
        super(deviceId, motorType);
        this.restoreFactoryDefaults();
        this.setIdleMode(idleMode);
        this.setInverted(isInverted);
        this.getPIDController().setOutputRange(sparkPidfConfig.kMinOutput, sparkPidfConfig.kMaxOutput);
        this.getPIDController().setP(sparkPidfConfig.kP);
        this.getPIDController().setI(sparkPidfConfig.kI);
        this.getPIDController().setD(sparkPidfConfig.kD);
        this.getPIDController().setFF(sparkPidfConfig.kFF);
    }
}
