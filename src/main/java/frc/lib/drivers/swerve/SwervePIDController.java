package frc.lib.drivers.swerve;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.structs.PIDFConfig;

public class SwervePIDController extends PIDController {
    public SwervePIDController(PIDFConfig config) {
        super(config.kP, config.kI, config.kD);
    }
}
