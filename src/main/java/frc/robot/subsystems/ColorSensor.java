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
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    private final Color kDefaultColor; // approximate values
    private final Color kNoteColor;

    private Color m_detectedColor;
    private ColorMatchResult m_colorMatch;
    private boolean noteDetected = false;

    public ColorSensor(I2C.Port port) {
        this.m_colorSensor = new ColorSensorV3(port);
        this.m_colorMatcher = new ColorMatch();

        this.kDefaultColor = this.m_colorSensor.getColor();
        this.kNoteColor = Colors.NOTE_COLOR; // found by testing
        System.out.println(this.kDefaultColor.red * 255.0);
        System.out.println(this.kDefaultColor.blue * 255.0);
        System.out.println(this.kDefaultColor.green * 255.0);

        this.m_colorMatcher.addColorMatch(kDefaultColor);
        this.m_colorMatcher.addColorMatch(kNoteColor);
    }

    @Override
    public void periodic() {
        this.m_detectedColor = this.m_colorSensor.getColor();
        this.m_colorMatch = this.m_colorMatcher.matchClosestColor(this.m_detectedColor);

        // this.noteDetected = this.m_colorMatch.color == kNoteColor ||
        // this.m_colorSensor.getProximity() > 35;
        this.noteDetected = this.m_colorSensor.getProximity() > 35;

        System.out.println(this.m_colorMatch.color.red * 255.0);
        System.out.println(this.m_colorMatch.color.blue * 255.0);
        System.out.println(this.m_colorMatch.color.green * 255.0);

        SmartDashboard.putBoolean("Note Detected", this.noteDetected);
        SmartDashboard.putNumber("IR", this.m_colorSensor.getIR());
        SmartDashboard.putNumber("Red", this.m_detectedColor.red * 255.0);
        SmartDashboard.putNumber("Blue", this.m_detectedColor.blue * 255.0);
        SmartDashboard.putNumber("Green", this.m_detectedColor.green * 255.0);
        SmartDashboard.putNumber("Proximity", this.m_colorSensor.getProximity());
    }

    public boolean isNoteDetected() {
        return this.noteDetected;
    }
}
