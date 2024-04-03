package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDCANdle;
import frc.robot.subsystems.LEDCANdle.CurrentState;
import frc.robot.subsystems.vision.Camera;

public class LEDCANdleDefaultCommand extends Command {

    private final LEDCANdle ledCANdleSubsystem;
    private final Camera cameraSubsystem;

    public LEDCANdleDefaultCommand(LEDCANdle ledCANdleSubsystem, Camera cameraSubsystem) {
        this.ledCANdleSubsystem = ledCANdleSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        addRequirements(this.ledCANdleSubsystem);
    }

    @Override
    public void initialize() {
        // this.ledCANdleSubsystem.rainbow();
    }

    @Override
    public void execute() {
        if (this.cameraSubsystem.getSpeakerTarget() != null) {
            if (this.ledCANdleSubsystem.getCurrentState() != CurrentState.BLUE) {
                this.ledCANdleSubsystem.setAllToBlue();
                return;
            }
        }
        if (this.ledCANdleSubsystem.getCurrentState() != CurrentState.RAINBOW) {
            this.ledCANdleSubsystem.rainbow();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}