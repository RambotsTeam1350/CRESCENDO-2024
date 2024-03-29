package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDH extends SubsystemBase {
    private final PowerDistribution pdh;

    private final ShuffleboardTab PDHTab;
    private final GenericEntry voltageEntry;
    private final GenericEntry temperatureEntry; // celsius
    private final GenericEntry totalCurrentEntry; // amps
    private final GenericEntry totalPowerEntry; // watts
    private final GenericEntry totalEnergyEntry; // joules

    private final ShuffleboardLayout channelCurrentsLayout;
    private final ArrayList<GenericEntry> channelCurrentEntries;

    public PDH() {
        this.pdh = new PowerDistribution(1, ModuleType.kRev);

        this.PDHTab = Shuffleboard.getTab("PDH");
        this.voltageEntry = this.PDHTab.add("Voltage", this.pdh.getVoltage()).getEntry();
        this.temperatureEntry = this.PDHTab.add("Temperature", this.pdh.getTemperature()).getEntry();
        this.totalCurrentEntry = this.PDHTab.add("Total Current", this.pdh.getTotalCurrent()).getEntry();
        this.totalPowerEntry = this.PDHTab.add("Total Power", this.pdh.getTotalPower()).getEntry();
        this.totalEnergyEntry = this.PDHTab.add("Total Energy", this.pdh.getTotalEnergy()).getEntry();

        this.channelCurrentsLayout = this.PDHTab.getLayout("Channels", BuiltInLayouts.kList);
        this.channelCurrentEntries = new ArrayList<GenericEntry>();
        for (int i = 0; i < 24; i++) {
            this.channelCurrentEntries
                    .add(this.channelCurrentsLayout.add("Channel " + i + " Current", this.pdh.getCurrent(i))
                            .getEntry());
        }
    }

    @Override
    public void periodic() {
        this.voltageEntry.setDouble(this.pdh.getVoltage());
        this.temperatureEntry.setDouble(this.pdh.getTemperature());
        this.totalCurrentEntry.setDouble(this.pdh.getTotalCurrent());
        this.totalPowerEntry.setDouble(this.pdh.getTotalPower());
        this.totalEnergyEntry.setDouble(this.pdh.getTotalEnergy());

        for (int i = 0; i < 24; i++) {
            this.channelCurrentEntries.get(i).setDouble(this.pdh.getCurrent(i));
        }
    }
}
