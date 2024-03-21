// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.structs.CANSparkPIDFConfig;
import frc.lib.structs.FFConfig;
import frc.lib.structs.PIDConfig;

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
  public static final class MotorFreeSpeeds {
    public static final double NEO_VORTEX = 6784.0;
    public static final double NEO_BRUSHLESS = 5676.0;
    public static final double NEO_550 = 11000.0;
  }

  public static final class Colors {
    public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;
    public static final Color NOTE_COLOR = new Color(132, 28, 94);
  }

  public static final class Controllers {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  public static final class Intake {
    public static final int POWER_MOTOR_ID = 1;
    public static final int ROTATION_MOTOR_ID = 2;
    public static final int ROTATION_THROUGH_BORE_ENCODER_DIO_PORT = 0;
    public static final int TOP_LIMIT_SWITCH_DIO_PORT = 4;

    public static final double ROTATION_THROUGH_BORE_ENCODER_POSITION_OFFSET = 0.323;
    // the intake treats 0 degrees as the intake being up inside the robot, and
    // moving out from the robot is + degrees
    public static final double UP_DEGREES = 0;
    public static final double STRAIGHT_DEGREES = 90;
    public static final double DOWN_DEGREES = 213;

    public static final double POWER_MOTOR_IN_DIRECTION = -1.0;
    public static final double POWER_MOTOR_OUT_DIRECTION = 1.0;

    public static final double POWER_MOTOR_MAX_RPM = MotorFreeSpeeds.NEO_BRUSHLESS;

    public static final CANSparkPIDFConfig POWER_MOTOR_SPARK_PIDF_CONFIG = new CANSparkPIDFConfig(0.000006, 0, 0,
        0, -1, 1);
    public static final PIDConfig ROTATION_MOTOR_PID_CONFIG = new PIDConfig(0.08 * 12.0, 0, 0); // we tuned this
    // slightly fudged kS value, but i had to account for there sometimes being
    // gravity
    public static final FFConfig POWER_MOTOR_FF_CONFIG = new FFConfig(0.132, 12.0 / 5740.0);
    public static final FFConfig ROTATION_MOTOR_FF_CONFIG = new FFConfig(0.2);
  }

  public static final class Shooter {
    public static final int POWER_MOTOR_1_ID = 3;
    public static final int POWER_MOTOR_2_ID = 4;
    public static final int ROTATION_MOTOR_ID = 5;
    public static final int ROTATION_THROUGH_BORE_ENCODER_DIO_PORT = 9;

    public static final double ROTATION_THROUGH_BORE_ENCODER_POSITION_OFFSET = 0.0;

    public static final double SPEED_MOTORS_MAX_RPM = MotorFreeSpeeds.NEO_VORTEX;

    public static final CANSparkPIDFConfig POWER_MOTOR_SPARK_PIDF_CONFIG = new CANSparkPIDFConfig(0.000006, 0, 0,
        0.000175, 0, 1);
    public static final PIDConfig ROTATION_MOTOR_PID_CONFIG = new PIDConfig(0.0001 * 12.0, 0, 0); // TODO: tune

    public static final FFConfig POWER_MOTOR_1_FF_CONFIG = new FFConfig(0.12, 12.0 / 6470.0);
    public static final FFConfig POWER_MOTOR_2_FF_CONFIG = new FFConfig(0.12, 12.0 / 6510.0);
    public static final FFConfig ROTATION_MOTOR_FF_CONFIG = new FFConfig(0.2); // TODO: find value

    public static final double VISION_MAX_DISTANCE = 1.7; // meters
  }

  public static final class Climber { // left and right is based on the perspective of the intake
    public static final int LEFT_MOTOR_ID = 6;
    public static final int RIGHT_MOTOR_ID = 7;

    public static final double UP_DIRECTION = -1.0;
    public static final double DOWN_DIRECTION = 1.0;

    public static final double MAX_RPM = MotorFreeSpeeds.NEO_BRUSHLESS;

    public static final CANSparkPIDFConfig LEFT_SPARK_PIDF_CONFIG = new CANSparkPIDFConfig(0.002, 0, 0, 0.0003, -1, 1);
    public static final CANSparkPIDFConfig RIGHT_SPARK_PIDF_CONFIG = new CANSparkPIDFConfig(0.002, 0, 0, 0.0006, -1, 1);
  }

  public static final class Swerve {
    public static final class FL {
      public static final int DRIVE_MOTOR_ID = 11;
      public static final int ANGLE_MOTOR_ID = 12;
      public static final int CANCODER_ID = 1;
      public static final double CANCODER_ALIGNMENT_OFFSET = -0.125488;
    }

    public static final class FR {
      public static final int DRIVE_MOTOR_ID = 21;
      public static final int ANGLE_MOTOR_ID = 22;
      public static final int CANCODER_ID = 2;
      public static final double CANCODER_ALIGNMENT_OFFSET = 0.346924;
    }

    public static final class BL {
      public static final int DRIVE_MOTOR_ID = 31;
      public static final int ANGLE_MOTOR_ID = 32;
      public static final int CANCODER_ID = 3;
      public static final double CANCODER_ALIGNMENT_OFFSET = -0.007812;
    }

    // apparently SPARKMAX ids over 40 cause issues so i just subtracted 6 from the
    // ids they would have been (41, 42)
    public static final class BR {
      public static final int DRIVE_MOTOR_ID = 35;
      public static final int ANGLE_MOTOR_ID = 36;
      public static final int CANCODER_ID = 4;
      public static final double CANCODER_ALIGNMENT_OFFSET = -0.427734;
    }

    public static final int PIGEON_ID = 10;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    // gear ratios:
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=46872600117549
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double ANGLE_MOTOR_GEAR_RATIO = 150.0 / 7.0;

    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double ANGLE_MOTOR_PCONVERSION = 2.0 * Math.PI / ANGLE_MOTOR_GEAR_RATIO;

    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double ANGLE_MOTOR_VCONVERSION = ANGLE_MOTOR_PCONVERSION / 60.0;

    public static final int DRIVE_MOTOR_SMART_LIMIT = 45;
    public static final int ANGLE_MOTOR_SMART_LIMIT = 25;

    // multiply by 12.0 because voltage control is used
    public static final PIDConfig DRIVE_MOTOR_PID_CONFIG = new PIDConfig(0.15 * 12.0); // TODO: not tuned at all
    public static final PIDConfig ANGLE_MOTOR_PID_CONFIG = new PIDConfig(0.4 * 12.0);

    public static final FFConfig DRIVE_MOTOR_FF_CONFIG = new FFConfig(0.118, 2.617);
    public static final FFConfig ANGLE_MOTOR_FF_CONFIG = new FFConfig(0.132);

    public static final double DRIVETRAIN_MAX_SPEED = 4.5 * 1.0;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.5 * Math.PI;

    // Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(20);
    public static final double WHEEL_BASE = Units.inchesToMeters(20);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    // Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED * 1.0;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3 * 0.5;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

    // Auton constraints
    public static final double AUTO_kP_TRANSLATION = 5;
    public static final double AUTO_kP_ROTATION = 5;

    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;

    public static final HolonomicPathFollowerConfig AUTO_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(AUTO_kP_TRANSLATION, 0.0, 0.0),
        new PIDConstants(AUTO_kP_ROTATION, 0.0, 0.0),
        AUTO_DRIVE_MAX_SPEED, // Max module speed, in m/s
        DRIVE_BASE_RADIUS,
        new ReplanningConfig());
  }

  public static final class Vision {
    public static final String TARGET_CAMERA = "photonvision";
    public static final int PIPELINE = 0;

    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(105);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(29.7); // 29.5

    public static final class FiducialIDs {
      public static final int SPEAKER = 4;
    }
  }
}
