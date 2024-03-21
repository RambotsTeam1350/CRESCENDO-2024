package frc.lib.drivers.uppermech;

import com.revrobotics.CANSparkMax;

public class ConfiguredCANSparkMax extends CANSparkMax {
    public ConfiguredCANSparkMax(int id, MotorType motorType, IdleMode idleMode, boolean inverted) {
        super(id, motorType);
        this.restoreFactoryDefaults();
        this.setIdleMode(idleMode);
        this.setInverted(inverted);
        this.burnFlash();
    }
}
