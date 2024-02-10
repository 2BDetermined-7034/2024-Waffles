// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Matrix;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 0.75;
  }

  public static final class AprilTags {


//    public static final HashMap<Integer, Pose3d> tagmap = new HashMap<>();
//    static {
//      Matrix tag1posmat = new Matrix(4, 4);
//      tag1posmat.data = new double[][] {
//              {-0.5,     -0.866025, 0, 6.808597},
//              {0.866025, -0.5,      0, -3.859403},
//              {0,        0,         1, 1.355852},
//              {0, 0, 0, 1}
//      };
//      tagmap.put(1, Matrix.toTransformations(tag1posmat));
//    }

    public static final List<AprilTag> layout;

    static {
      try {
        layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTags();
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }
  }

  public static final class Shooter {
    public static final int shooterVelocityTalonID = 6  ;
    public static final int shooterAngleTalonID = 5;
    public static final int shooterNeo550ID = 9;
    public static final double angleGearRatio = (72.0 / 18.0) * (4.0 / 1.0);
    public static final double angleAtHorizon = 52;
  }
}
