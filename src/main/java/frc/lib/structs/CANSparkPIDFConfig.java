package frc.lib.structs;

public class CANSparkPIDFConfig extends PIDConfig {
    public double kFF;
    public double kMinOutput;
    public double kMaxOutput;

    public CANSparkPIDFConfig(double kP, double kI, double kD, double kFF, double kMinOutput, double kMaxOutput) {
        super(kP, kI, kD);
        this.kFF = kFF;
        this.kMinOutput = kMinOutput;
        this.kMaxOutput = kMaxOutput;
    }
}
