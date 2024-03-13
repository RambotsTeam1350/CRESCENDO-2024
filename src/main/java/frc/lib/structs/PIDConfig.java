package frc.lib.structs;

public class PIDConfig {
    public final double kP;
    public final double kI;
    public final double kD;

    public PIDConfig(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDConfig(double kP, double kI) {
        this(kP, kI, 0);
    }

    public PIDConfig(double kP) {
        this(kP, 0, 0);
    }
}
