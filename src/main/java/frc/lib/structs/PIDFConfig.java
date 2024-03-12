package frc.lib.structs;

public class PIDFConfig {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    public PIDFConfig(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public PIDFConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    public PIDFConfig(double kP, double kI) {
        this(kP, kI, 0, 0);
    }

    public PIDFConfig(double kP) {
        this(kP, 0, 0, 0);
    }
}
