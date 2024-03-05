// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain;

  private final CommandXboxController m_driverController;

  private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.m_drivetrain = new Drivetrain();
    this.m_driverController = new CommandXboxController(Constants.Controllers.Driver.PORT);

    m_drivetrain.setDefaultCommand(new SwerveDrive(this.m_drivetrain, this.m_driverController.getHID()));

    this.m_autoChooser = AutoBuilder.buildAutoChooser("Default Auto");
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    registerNamedCommands();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    this.m_driverController.start().onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));
    this.m_driverController.povUp().toggleOnTrue(Commands.startEnd(
        // command start
        () -> this.m_drivetrain.swerveDrive(
            0.1,
            0,
            0,
            !this.m_driverController.getHID().getBButtonPressed(),
            new Translation2d(),
            true),
        // command end
        () -> this.m_drivetrain.stopModules(),
        // required subsystem
        this.m_drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    this.m_drivetrain.resetAllEncoders();
    this.m_drivetrain.setHeading(0);
    return m_autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Stop Modules", new InstantCommand(() -> m_drivetrain.stopModules()));
  }

  public Drivetrain getDrivetrain() {
    return this.m_drivetrain;
  }
}
