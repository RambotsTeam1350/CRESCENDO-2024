// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers.swerve;

import com.revrobotics.CANSparkMax;

public class SwerveCANSparkMAX extends CANSparkMax {
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * 
     * @param deviceId          The device ID.
     * @param motorType         The motor type (Brushed/Brushless).
     * @param idleMode          The idle mode (kBrake/kCoast).
     * @param smartCurrentLimit The current limit.
     * @param isInverted        The invert type of the motor.
     */
    public SwerveCANSparkMAX(int deviceId, MotorType motorType, IdleMode idleMode, int smartCurrentLimit,
            boolean isInverted) {
        super(deviceId, motorType);
        this.restoreFactoryDefaults();
        this.setSmartCurrentLimit(smartCurrentLimit);
        this.setInverted(isInverted);
        this.setIdleMode(idleMode);
        this.burnFlash();
    }
}