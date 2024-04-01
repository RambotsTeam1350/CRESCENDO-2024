package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDCANdle extends SubsystemBase {
    private final CANdle CANdle;
    private final CANdleConfiguration CANdleConfiguration;

    public LEDCANdle() {
        this.CANdle = new CANdle(5);
        this.CANdleConfiguration = new CANdleConfiguration();
        this.CANdleConfiguration.statusLedOffWhenActive = false;
        this.CANdleConfiguration.disableWhenLOS = false;
        this.CANdleConfiguration.stripType = LEDStripType.RGB;
        this.CANdleConfiguration.brightnessScalar = 1;
        this.CANdleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;

        this.CANdle.configAllSettings(this.CANdleConfiguration, 100);

        // this.setAllLedToColor(0, 0, 255);
        this.rainbow();
    }

    public void setAllLedToColor(int r, int g, int b) {
        this.CANdle.setLEDs(r, g, b);
        this.CANdle.modulateVBatOutput(1);
    }

    public void rainbow() {
        this.CANdle.animate(new RainbowAnimation(0.4, 0.25, 23));
    }
}