// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.intake.GrabNote;
import frc.robot.commands.intake.RotateIntakeDownAndRunMotorsInUntilNoteGrabbed;
import frc.robot.commands.intake.RotateIntakeToAngle;
import frc.robot.commands.shooter.FeedNote;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Drivetrain;

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
  private final Climber m_climber;
  private final Intake m_intake;
  private final Shooter m_shooter;
  @SuppressWarnings("unused")
  private final ColorSensor m_colorSensor;

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.m_drivetrain = new Drivetrain();
    this.m_climber = new Climber();
    this.m_intake = new Intake();
    this.m_shooter = new Shooter();
    this.m_colorSensor = new ColorSensor(Constants.Colors.COLOR_SENSOR_PORT);

    this.m_driverController = new CommandXboxController(Constants.Controllers.DRIVER_PORT);
    this.m_operatorController = new CommandXboxController(Constants.Controllers.OPERATOR_PORT);

    this.registerNamedCommands(); // do not move (https://pathplanner.dev/pplib-named-commands.html)

    this.m_drivetrain.setDefaultCommand(new SwerveDrive(this.m_drivetrain, this.m_driverController.getHID()));

    this.configureBindings();

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
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
    this.m_driverController.start().onTrue(new InstantCommand(m_drivetrain::zeroHeading, this.m_drivetrain));
    this.m_operatorController.povUp().whileTrue(new ClimbUp(this.m_climber));
    this.m_operatorController.povDown().whileTrue(new ClimbDown(this.m_climber));
    this.m_operatorController.povLeft()
        .whileTrue(new RotateIntakeToAngle(this.m_intake, Constants.Intake.UP_ABSOLUTE_ENCODER_VALUE));
    // this.m_operatorController.povRight()
    // .whileTrue(new RotateIntakeToAngle(this.m_intake,
    // Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE));
    this.m_operatorController.povRight()
        .whileTrue(new RotateIntakeDownAndRunMotorsInUntilNoteGrabbed(this.m_intake, this.m_colorSensor));
    this.m_operatorController.a().whileTrue(new RunShooter(this.m_shooter));
    this.m_operatorController.b().whileTrue(new FeedNote(this.m_intake));
    this.m_operatorController.x().whileTrue(new GrabNote(this.m_intake, this.m_colorSensor));
    // this.m_operatorController.y()
    // .whileTrue(new RotateIntakeToAngle(this.m_intake,
    // Constants.Intake.STRAIGHT_ABSOLUTE_ENCODER_VALUE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    this.m_drivetrain.resetAllEncoders();
    return this.m_autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Stop Modules", new InstantCommand(m_drivetrain::stopModules));
    NamedCommands.registerCommand("Intake Up",
        new RotateIntakeToAngle(this.m_intake, Constants.Intake.UP_ABSOLUTE_ENCODER_VALUE));
    NamedCommands.registerCommand("Intake Down",
        new RotateIntakeToAngle(this.m_intake, Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE));
    NamedCommands.registerCommand("Grab Note",
        new GrabNote(this.m_intake, this.m_colorSensor));
    NamedCommands.registerCommand("Run Shooter", new RunShooter(m_shooter));
    NamedCommands.registerCommand("Feed Note", new FeedNote(this.m_intake));
  }

  public Drivetrain getDrivetrain() {
    return this.m_drivetrain;
  }
}
