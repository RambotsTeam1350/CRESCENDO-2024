package frc.lib.drivers.swerve;

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
public class SwerveSimpleMotorFeedforward extends SimpleMotorFeedforward {
    public SwerveSimpleMotorFeedforward(FFConfig config) {
        super(config.kS, config.kV, config.kA);
    }
}
