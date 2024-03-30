package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.vision.Camera;

public class SetLEDs extends Command {
    private final LED ledSubsystem;
    private final Camera cameraSubsystem;

    public SetLEDs(LED ledSubsystem, Camera cameraSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        addRequirements(this.ledSubsystem);
        // DO NOT ADD REQUIREMENTS, THIS COMMAND IS NOT SUPPOSED TO BE INTERRUPTED
    }

    public void execute() {
        if (this.cameraSubsystem.hasTarget()) {
            this.ledSubsystem.setLEDs();
        }
    }
}
