// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Controllers {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  public static final class Swerve {
    public static final class FL {
      public static final int DRIVE_MOTOR_ID = 11;
      public static final int ANGLE_MOTOR_ID = 12;
      public static final int CANCODER_ID = 1;
      public static final double CANCODER_ALIGNMENT_OFFSET = 0.059;
    }

    public static final class FR {
      public static final int DRIVE_MOTOR_ID = 21;
      public static final int ANGLE_MOTOR_ID = 22;
      public static final int CANCODER_ID = 2;
      public static final double CANCODER_ALIGNMENT_OFFSET = -0.396;
    }

    public static final class BL {
      public static final int DRIVE_MOTOR_ID = 31;
      public static final int ANGLE_MOTOR_ID = 32;
      public static final int CANCODER_ID = 3;
      public static final double CANCODER_ALIGNMENT_OFFSET = -0.329;
    }

    // apparently SPARKMAX ids over 40 cause issues so i just subtracted 6 from the
    // ids they would have been (41, 42)
    public static final class BR {
      public static final int DRIVE_MOTOR_ID = 35;
      public static final int ANGLE_MOTOR_ID = 36;
      public static final int CANCODER_ID = 4;
      public static final double CANCODER_ALIGNMENT_OFFSET = 0.286;
    }

    public static final int PIGEON_ID = 10;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    // gear ratios:
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=46872600117549
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double ANGLE_MOTOR_GEAR_RATIO = 150.0 / 7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double ANGLE_MOTOR_PCONVERSION = 2 * Math.PI / ANGLE_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double ANGLE_MOTOR_VCONVERSION = ANGLE_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.5; // TODO: tune

    public static final double DRIVETRAIN_MAX_SPEED = 4.0;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.5 * Math.PI;

    // Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(25.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(25.5);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

    // Auton constraints
    public static final double AUTO_kP_TRANSLATION = 0.4;
    public static final double AUTO_kP_ROTATION = 2.4;

    public static final HolonomicPathFollowerConfig AUTO_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(AUTO_kP_TRANSLATION, 0.0, 0.0),
        new PIDConstants(AUTO_kP_ROTATION, 0.0, 0.0),
        DRIVETRAIN_MAX_SPEED, // Max module speed, in m/s
        DRIVE_BASE_RADIUS,
        new ReplanningConfig());

    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1.5;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;
  }
}
