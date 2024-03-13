package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;

public class IntakeDown extends Command {
    private final Intake m_intake;

    public IntakeDown(Intake intake) {
        this.m_intake = intake;
        addRequirements(this.m_intake);
    }

    @Override
    public void initialize() {
        this.m_intake.setRotationAngleSetpoint(Constants.Intake.DOWN_DEGREES); // lower intake
    }

    @Override
    public boolean isFinished() {
        // there's no limit switch for the lowering so just pray that the setpoint works
        // 🙏🙏🙏🙏
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopRotationMotor();
    }
}
