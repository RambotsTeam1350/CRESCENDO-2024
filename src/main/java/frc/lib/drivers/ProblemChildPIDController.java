package frc.lib.drivers;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.structs.PIDConfig;

public class ProblemChildPIDController extends PIDController {
    public ProblemChildPIDController(PIDConfig config) {
        super(config.kP, config.kI, config.kD);
    }
}
