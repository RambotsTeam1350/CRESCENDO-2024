package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Camera;

public class LEDCANdle extends SubsystemBase {
    private final CANdle CANdle;
    private final CANdleConfiguration CANdleConfiguration;

    private final Camera cameraSubsystem;

    private CurrentState currentState = CurrentState.OFF;

    private boolean taken = false;

    public enum CurrentState {
        OFF,
        BLUE,
        ORANGE,
        GREEN,
        RAINBOW
    }

    public LEDCANdle(Camera cameraSubsystem) {
        this.CANdle = new CANdle(5);
        this.CANdleConfiguration = new CANdleConfiguration();
        this.CANdleConfiguration.statusLedOffWhenActive = false;
        this.CANdleConfiguration.disableWhenLOS = false;
        this.CANdleConfiguration.stripType = LEDStripType.RGB;
        this.CANdleConfiguration.brightnessScalar = 1;
        this.CANdleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;

        this.CANdle.configAllSettings(this.CANdleConfiguration, 100);

        this.cameraSubsystem = cameraSubsystem;

        this.setAllToGreen();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("LED State", this.getCurrentState().toString());
        SmartDashboard.putBoolean("taken", taken);

        if (this.taken) {
            return;
        }

        if (this.cameraSubsystem.getSpeakerTarget() != null) {
            if (this.getCurrentState() != CurrentState.BLUE) {
                this.setAllToBlue();
            }
        } else {
            if (this.getCurrentState() != CurrentState.RAINBOW) {
                this.rainbow();
            }
        }

    }

    public void toggleTaken() {
        this.taken = !this.taken;
    }

    public boolean isTaken() {
        return this.taken;
    }

    public void setAllToColor(int r, int g, int b) {
        this.CANdle.clearAnimation(0);
        this.CANdle.setLEDs(r, g, b);
        this.CANdle.modulateVBatOutput(1);
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
        this.CANdle.animate(new RainbowAnimation(0.4, 0.5, 23), 0);
        this.currentState = CurrentState.RAINBOW;
    }

    public void off() {
        this.setAllToColor(0, 0, 0);
    }

    public CurrentState getCurrentState() {
        return this.currentState;
    }
}