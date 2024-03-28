package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Colors;

public class ColorSensor extends SubsystemBase {
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;
    private final Color kDefaultColor; // approximate values
    private final Color kNoteColor;

    private Color detectedColor;
    private ColorMatchResult colorMatch;
    private boolean noteDetected = false;

    public ColorSensor(I2C.Port port) {
        this.colorSensor = new ColorSensorV3(port);
        this.colorMatcher = new ColorMatch();

        this.kDefaultColor = this.colorSensor.getColor();
        this.kNoteColor = Colors.NOTE_COLOR; // found by testing

        this.colorMatcher.addColorMatch(kDefaultColor);
        this.colorMatcher.addColorMatch(kNoteColor);
    }

    @Override
    public void periodic() {
        this.detectedColor = this.colorSensor.getColor();
        this.colorMatch = this.colorMatcher.matchClosestColor(this.detectedColor);

        // this.noteDetected = this.m_colorMatch.color == kNoteColor ||
        // this.m_colorSensor.getProximity() > 35;
        this.noteDetected = this.colorSensor.getProximity() > 37;

        SmartDashboard.putBoolean("Note Detected", this.noteDetected);
        // SmartDashboard.putNumber("IR", this.m_colorSensor.getIR());
        // SmartDashboard.putNumber("Red", this.m_detectedColor.red * 255.0);
        // SmartDashboard.putNumber("Blue", this.m_detectedColor.blue * 255.0);
        // SmartDashboard.putNumber("Green", this.m_detectedColor.green * 255.0);
        SmartDashboard.putNumber("Proximity", this.colorSensor.getProximity());
    }

    public boolean isNoteDetected() {
        return this.noteDetected;
    }
}
