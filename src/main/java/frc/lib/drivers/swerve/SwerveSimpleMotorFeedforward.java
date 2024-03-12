package frc.lib.drivers.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.structs.FFConfig;

public class SwerveSimpleMotorFeedforward extends SimpleMotorFeedforward {
    public SwerveSimpleMotorFeedforward(FFConfig config) {
        super(config.kS, config.kV, config.kA);
    }
}
