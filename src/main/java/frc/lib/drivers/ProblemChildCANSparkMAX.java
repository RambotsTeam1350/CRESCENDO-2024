// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class ProblemChildCANSparkMAX extends CANSparkMax {
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * 
     * @param deviceId   The device ID.
     * @param m          The motor type (Brushed/Brushless).
     * @param mode       The idle mode (kBrake/kCoast).
     * @param limit      The current limit.
     * @param isInverted The invert type of the motor.
     */
    public ProblemChildCANSparkMAX(int deviceId, MotorType m, IdleMode mode, int limit, boolean isInverted) {
        super(deviceId, m);
        this.restoreFactoryDefaults();
        this.setSmartCurrentLimit(limit);
        this.setInverted(isInverted);
        this.setIdleMode(mode);
        this.burnFlash();
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }
}