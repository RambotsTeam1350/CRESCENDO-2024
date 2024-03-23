// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.routines.IntakeNote;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.intake.FeedNote;
import frc.robot.commands.intake.ForceSlowGrabNote;
import frc.robot.commands.intake.GrabNote;
import frc.robot.commands.intake.RotateIntakeToAngle;
import frc.robot.commands.shooter.RunStopShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.shooter.AutoRotateShooterToSpeakerAngle;
import frc.robot.commands.shooter.RotateShooterToAngle;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.intake.IntakePower;
import frc.robot.subsystems.intake.IntakeRotation;
import frc.robot.subsystems.shooter.ShooterPower;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Camera;

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
  private final Camera cameraSubsystem;

  private final Drivetrain drivetrainSubsystem;

  private final Climber climberSubsystem;

  private final IntakeRotation intakeRotationSubsystem;
  private final IntakePower intakePowerSubsystem;

  private final ShooterRotation shooterRotationSubsystem;
  private final ShooterPower shooterPowerSubsystem;

  private final ColorSensor colorSensorSubsystem;

  private final LED ledSubsystem;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.cameraSubsystem = new Camera();

    this.drivetrainSubsystem = new Drivetrain(this.cameraSubsystem);

    this.climberSubsystem = new Climber();

    this.intakeRotationSubsystem = new IntakeRotation();
    this.intakePowerSubsystem = new IntakePower();

    this.shooterRotationSubsystem = new ShooterRotation();
    this.shooterPowerSubsystem = new ShooterPower();

    this.colorSensorSubsystem = new ColorSensor(Constants.Colors.COLOR_SENSOR_PORT);

    this.ledSubsystem = new LED();

    this.driverController = new CommandXboxController(Constants.Controllers.DRIVER_PORT);
    this.operatorController = new CommandXboxController(Constants.Controllers.OPERATOR_PORT);

    this.registerNamedCommands(); // do not move (https://pathplanner.dev/pplib-named-commands.html)

    this.drivetrainSubsystem
        .setDefaultCommand(new SwerveDrive(this.drivetrainSubsystem, this.driverController.getHID()));

    this.configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    this.driverController.start()
        .onTrue(new InstantCommand(drivetrainSubsystem::zeroHeading, this.drivetrainSubsystem));
    this.operatorController.povUp().whileTrue(new ClimbUp(this.climberSubsystem));
    this.operatorController.povDown().whileTrue(new ClimbDown(this.climberSubsystem));
    this.operatorController.povLeft()
        .toggleOnTrue(RotateIntakeToAngle.createIntakeUpCommand(this.intakeRotationSubsystem));
    this.operatorController.y()
        .toggleOnTrue(RotateIntakeToAngle.createIntakeStraightCommand(this.intakeRotationSubsystem));
    // this.m_operatorController.povRight()
    // .whileTrue(new RotateIntakeToAngle(this.m_intake,
    // Constants.Intake.DOWN_ABSOLUTE_ENCODER_VALUE));

    // this.operatorController.povRight()
    // .toggleOnTrue(new
    // RotateIntakeDownAndRunPowerMotorsInUntilNoteDetected(this.intakeRotationSubsystem,
    // this.colorSensorSubsystem)
    // .andThen(
    // new RotateIntakeToAngle(this.intakeRotationSubsystem,
    // Constants.Intake.UP_ABSOLUTE_ENCODER_VALUE)));

    this.operatorController.povRight()
        .toggleOnTrue(
            new IntakeNote(this.intakeRotationSubsystem, this.intakePowerSubsystem, this.colorSensorSubsystem));
    this.operatorController.a().whileTrue(new RunStopShooter(this.shooterPowerSubsystem));
    this.operatorController.leftBumper().whileTrue(new ForceSlowGrabNote(this.intakePowerSubsystem));
    this.operatorController.rightBumper().whileTrue(new FeedNote(this.intakePowerSubsystem));
    this.operatorController.x().toggleOnTrue(RotateIntakeToAngle.createIntakeDownCommand(this.intakeRotationSubsystem));
    this.operatorController.back().onTrue(new FeedNote(this.intakePowerSubsystem).withTimeout(0.125));
    this.operatorController.b().toggleOnTrue(
        new AutoRotateShooterToSpeakerAngle(this.shooterRotationSubsystem, this.cameraSubsystem, this.ledSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    this.drivetrainSubsystem.resetAllEncoders();
    // return new PathPlannerAuto("Test Auto");
    return this.autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Zero Heading", new InstantCommand(this.drivetrainSubsystem::zeroHeading));
    NamedCommands.registerCommand("Stop Modules", new InstantCommand(drivetrainSubsystem::stopModules));
    NamedCommands.registerCommand("Intake Up", RotateIntakeToAngle.createIntakeUpCommand(this.intakeRotationSubsystem));
    NamedCommands.registerCommand("Intake Down",
        RotateIntakeToAngle.createIntakeDownCommand(this.intakeRotationSubsystem));
    NamedCommands.registerCommand("Intake Note",
        new IntakeNote(intakeRotationSubsystem, intakePowerSubsystem, colorSensorSubsystem));
    NamedCommands.registerCommand("Grab Note",
        new GrabNote(this.intakePowerSubsystem, this.colorSensorSubsystem));
    NamedCommands.registerCommand("Run Shooter", new RunShooter(shooterPowerSubsystem).withTimeout(1.5));
    NamedCommands.registerCommand("Spool Up Shooter", new RunShooter(shooterPowerSubsystem).withTimeout(0.1));
    NamedCommands.registerCommand("Stop Shooter", new StopShooter(this.shooterPowerSubsystem));
    NamedCommands.registerCommand("Feed Note", new FeedNote(this.intakePowerSubsystem).withTimeout(2));
    NamedCommands.registerCommand("Stop Feed Note", new InstantCommand(this.intakePowerSubsystem::stopMotor));
  }

  public Drivetrain getDrivetrainSubsystem() {
    return this.drivetrainSubsystem;
  }
}
