// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;

public class SwerveDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final XboxController m_controller;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(Drivetrain drivetrain, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = drivetrain;
    this.m_controller = controller;
    addRequirements(this.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_drivetrain.controllerDrive(
        -this.m_controller.getLeftY(),
        -this.m_controller.getLeftX(),
        -this.m_controller.getRightX(),
        !this.m_controller.getRawButton(XboxController.Button.kB.value),
        this.m_controller.getRawButton(XboxController.Button.kA.value),
        new Translation2d(),
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
