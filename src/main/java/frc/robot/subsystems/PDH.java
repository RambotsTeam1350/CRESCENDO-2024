package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDH extends SubsystemBase {
    private final PowerDistribution pdh;

    public PDH() {
        this.pdh = new PowerDistribution(0, ModuleType.kRev);
    }
}
