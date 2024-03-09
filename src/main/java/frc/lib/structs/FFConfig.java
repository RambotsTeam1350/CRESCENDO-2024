package frc.lib.structs;

public class FFConfig {
    public final double kS;
    public final double kV;
    public final double kA;

    public FFConfig(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public FFConfig(double kS, double kV) {
        this(kS, kV, 0);
    }

    public FFConfig(double kS) {
        this(kS, 0, 0);
    }
}
