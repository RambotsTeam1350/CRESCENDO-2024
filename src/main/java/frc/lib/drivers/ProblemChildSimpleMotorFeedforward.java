package frc.lib.drivers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.structs.FFConfig;

public class ProblemChildSimpleMotorFeedforward extends SimpleMotorFeedforward {
    public ProblemChildSimpleMotorFeedforward(FFConfig config) {
        super(config.kS, config.kV, config.kA);
    }
}
