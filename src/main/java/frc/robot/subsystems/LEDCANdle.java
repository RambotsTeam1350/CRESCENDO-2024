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
    private CurrentState currentState;

    public enum CurrentState {
        BLUE,
        ORANGE,
        GREEN,
        RAINBOW
    }

    public LEDCANdle() {
        this.CANdle = new CANdle(5);
        this.CANdleConfiguration = new CANdleConfiguration();
        this.CANdleConfiguration.statusLedOffWhenActive = false;
        this.CANdleConfiguration.disableWhenLOS = false;
        this.CANdleConfiguration.stripType = LEDStripType.RGB;
        this.CANdleConfiguration.brightnessScalar = 0.2;
        this.CANdleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;

        this.CANdle.configAllSettings(this.CANdleConfiguration, 100);
    }

    public void setAllToColor(int r, int g, int b) {
        this.CANdle.setLEDs(r, g, b);
        this.CANdle.modulateVBatOutput(0.9);
    }

    public void setAllToGreen() {
        this.setAllToColor(0, 255, 0);
        this.currentState = CurrentState.GREEN;
    }

    public void setAllToBlue() {
        this.setAllToColor(0, 0, 255);
        this.currentState = CurrentState.BLUE;
    }

    public void setAllToOrange() {
        this.setAllToColor(255, 165, 0);
        this.currentState = CurrentState.ORANGE;
    }

    public void rainbow() {
        this.CANdle.animate(new RainbowAnimation(0.4, 0.5, 23));
        this.currentState = CurrentState.RAINBOW;
    }

    public CurrentState getCurrentState() {
        return this.currentState;
    }
}