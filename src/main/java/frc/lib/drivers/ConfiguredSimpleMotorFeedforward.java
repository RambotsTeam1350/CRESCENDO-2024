package frc.lib.drivers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.structs.FFConfig;

/**
 * Creates a new {@link SimpleMotorFeedforward} with the necessary
 * configurations.
 * 
 * @param config The {@link FFConfig} constants to configure the
 *               {@link SimpleMotorFeedforward}
 *               with.
 */
public class ConfiguredSimpleMotorFeedforward extends SimpleMotorFeedforward {
    public ConfiguredSimpleMotorFeedforward(FFConfig config) {
        super(config.kS, config.kV, config.kA);
    }
}
