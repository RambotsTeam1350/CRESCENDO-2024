// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers.swerve;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class SwerveCANSparkMAX extends CANSparkMax {
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * 
     * @param deviceId   The device ID.
     * @param m          The motor type (Brushed/Brushless).
     * @param mode       The idle mode (kBrake/kCoast).
     * @param limit      The current limit.
     * @param isInverted The invert type of the motor.
     */
    public SwerveCANSparkMAX(int deviceId, MotorType m, IdleMode mode, int limit, boolean isInverted) {
        super(deviceId, m);
        this.restoreFactoryDefaults();
        this.setSmartCurrentLimit(limit);
        this.setInverted(isInverted);
        this.setIdleMode(mode);
        this.burnFlash();
    }
}