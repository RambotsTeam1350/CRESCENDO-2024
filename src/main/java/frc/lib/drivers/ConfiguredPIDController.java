package frc.lib.drivers;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.structs.PIDConfig;

/**
 * Creates a new PIDController with the necessary configurations.
 * 
 * @param config The {@link PIDConfig} constants to configure the PID
 *               Controller with.
 */
public class ConfiguredPIDController extends PIDController {
    public ConfiguredPIDController(PIDConfig config) {
        super(config.kP, config.kI, config.kD);
    }
}
