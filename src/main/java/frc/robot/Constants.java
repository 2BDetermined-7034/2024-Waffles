// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Matrix;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

import java.util.HashMap;

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

  //ClimbSubsystem Constants

  public static final class PhotonVision {
    public static final String backCameraName = "ShooterCam";
    public static final String frontCameraName = "IntakeCam";
  }
  public static final class Climb {
    public static final int motorAID = 255;
    public static final int motorBID = 256;
    public static final double MAX_CLIMB_VELOCITY = 0.5;
    public static final double CLIMB_MOTOR_TARGET_POSITION_REVOLUTIONS = 2.0;
    public static final double CLIMB_MOTOR_START_POSITION_REVOLUTIONS = 0.0; //I don't think we need this btw
  }
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

  public static class Speeds {
    public static final double TURN_SPEED = 2.5; //aaryan pls dont set it to 5
    /** 1 is <b><i>highly<i/><b/> recommended!*/
    public static final double MOVEMENT_SPEED = 1;
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


    public static final HashMap<Integer, Pose3d> tagmap = new HashMap<>();
    static {
      Matrix tag1posmat = new Matrix(4, 4);
      tag1posmat.data = new double[][] {
              {-0.5,     -0.866025, 0, 6.808597},
              {0.866025, -0.5,      0, -3.859403},
              {0,        0,         1, 1.355852},
              {0, 0, 0, 1}
      };
      tagmap.put(1, Matrix.toTransformations(tag1posmat));
    }
  }
}
