package frc.lib.drivers.uppermech;

import com.revrobotics.CANSparkMax;

public class ConfiguredCANSparkMax extends CANSparkMax {
    public ConfiguredCANSparkMax(int id, MotorType motorType, IdleMode idleMode, int currentLimit, boolean inverted) {
        super(id, motorType);
        this.restoreFactoryDefaults();
        this.setIdleMode(idleMode);
        this.setSmartCurrentLimit(currentLimit);
        this.setInverted(inverted);
        this.burnFlash();
    }
}
