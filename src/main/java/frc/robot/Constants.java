// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@SuppressWarnings("unused")
public final class Constants {

  public enum LedColor {
    BALANCING(0.57), // PINK
    DOCKED(0.77), // GREEN
    GRABBER_OPEN(0.67), // GOLD
    GRABBER_CLOSED(0.93), // WHITE
    BLUE_ALLIANCE(0.87), // BLUE
    RED_ALLIANCE(0.61); // RED

    private final double color;

    LedColor(double color) {
      this.color = color;
    }

    public double color() {
      return color;
    }
  }

  public enum LimelightDirections {
    GRID_SIDE(180),
    SUBSTATION_SIDE(0);

    private final int angle;

    LimelightDirections(int angle) {
      this.angle = angle;
    }

    public int angle() {
      return angle;
    }
  }

  public static final class DrivetrainConstants {

    public static final double DRIFT_DEADBAND = 0.1;
    public static final double ROTATION_DEADBAND = 0.006;

    public static final int BACK_RIGHT_SPEED_ID = 4;
    public static final int BACK_RIGHT_ANGLE_ID = 3;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0.829 - 0.25;

    public static final int BACK_LEFT_SPEED_ID = 2;
    public static final int BACK_LEFT_ANGLE_ID = 1;
    public static final double BACK_LEFT_ENCODER_OFFSET = 1 - 0.512;

    public static final int FRONT_RIGHT_SPEED_ID = 6;
    public static final int FRONT_RIGHT_ANGLE_ID = 5;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.142;

    public static final int FRONT_LEFT_SPEED_ID = 8;
    public static final int FRONT_LEFT_ANGLE_ID = 7;
    public static final double FRONT_LEFT_ENCODER_OFFSET = 0.082 + 0.25;

    public static final Translation2d FRONT_LEFT_LOC = new Translation2d(0.265, 0.265);
    public static final Translation2d FRONT_RIGHT_LOC = new Translation2d(0.265, -0.265);
    public static final Translation2d BACK_LEFT_LOC = new Translation2d(-0.265, 0.265);
    public static final Translation2d BACK_RIGHT_LOC = new Translation2d(-0.265, -0.265);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(FRONT_LEFT_LOC, FRONT_RIGHT_LOC, BACK_LEFT_LOC, BACK_RIGHT_LOC);
  }

  public static final class WheelConstants {
    public static final double ROTATIONS_TO_METERS = 0.245 / 5.08;
  }

  public static final class ArmConstants {
    public static final int PIVOT_MOTOR_ID = 10;
    public static final int TELESCOPE_MOTOR_ID = 11;
    public static final double pivotOffset = 0.687;

    public static final double startingPivotG = 0.8; // 2.01;
    public static final double pivotGPerTelescopeMeter = 0.8;
    public static final double telescopeRotationToMeters = -0.09 / 32768;

    public static final double telescopeOuterSetpoint = 0.9;
    public static final double telescopeMiddleSetpoint = 0.3;
    public static final double telescopeBottomSetpoint = 0.3;
    public static final double telescopeSubstationSetpoint = 0.568;

    public static final double elevatorUpPivotDownPosition = 0.81;
    public static final double pivotDownPosition = 0.83;

    public static final double topConePosition = 0.015;
    public static final double middleConePosition = 0.015;
    public static final double bottomConePosition = 0.015;
    public static final double substationConePosition = 0.015;
  }

  public static final class GrabberConstants {
    public static final int PNEUMATICS_HUB_ID = 23;
    public static final int LEFT_SOLENOID_REVERSE = 3;
    public static final int LEFT_SOLENOID_FORWARD = 2;

    public static final int RIGHT_SOLENOID_REVERSE = 1;
    public static final int RIGHT_SOLENOID_FORWARD = 0;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 9;
  }

  public static final class DriverConstants {
    public static final double speedMultiplier = 3;
    public static final double angleMultiplier = 3;
  }

  public static final class AutoConstants {

    public static final double autoTelescopeIn = 0.45;
  }

  public static final class LedConstants {
    public static final int BLINKIN_CHANNEL = 0;
  }

  public enum ConePosition {
    TOP,
    MIDDLE,
    BOTTOM,
    SUBSTATION
  }
}
