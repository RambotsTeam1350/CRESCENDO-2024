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
        this.m_intake.setRotationMotorVoltageSetpoint(-8);
        // this.m_intake.setRotationAngleSetpoint(Constants.Intake.DOWN_DEGREES); //
        // lower intake
    }

    @Override
    public boolean isFinished() {
        return this.m_intake.getRotationAbsoluteEncoder() >= Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopRotationMotor();
    }
}
