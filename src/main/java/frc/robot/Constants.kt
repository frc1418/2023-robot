// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@Suppress("unused")
class Constants {
    enum class LedColor(val color: Double) {
        // RED
        BALANCING(0.57),

        // PINK
        DOCKED(0.77),

        // GREEN
        GRABBER_OPEN(0.67),

        // GOLD
        GRABBER_CLOSED(0.93),

        // WHITE
        BLUE_ALLIANCE(0.87),

        // BLUE
        RED_ALLIANCE(0.61)
    }

    enum class LimelightDirections(val angle: Double) {
        GRID_SIDE(180.0),
        SUBSTATION_SIDE(0.0)
    }

    enum class WheelLocation {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    object DrivetrainConstants {
        const val DRIFT_DEADBAND = 0.1
        const val ROTATION_DEADBAND = 0.006
        const val BACK_RIGHT_SPEED_ID = 4
        const val BACK_RIGHT_ANGLE_ID = 3
        const val BACK_RIGHT_ENCODER_OFFSET = 0.829 - 0.25
        const val BACK_LEFT_SPEED_ID = 2
        const val BACK_LEFT_ANGLE_ID = 1
        const val BACK_LEFT_ENCODER_OFFSET = 1 - 0.512
        const val FRONT_RIGHT_SPEED_ID = 6
        const val FRONT_RIGHT_ANGLE_ID = 5
        const val FRONT_RIGHT_ENCODER_OFFSET = 0.142
        const val FRONT_LEFT_SPEED_ID = 8
        const val FRONT_LEFT_ANGLE_ID = 7
        const val FRONT_LEFT_ENCODER_OFFSET = 0.082 + 0.25
        private val FRONT_LEFT_LOC = Translation2d(0.265, 0.265)
        private val FRONT_RIGHT_LOC = Translation2d(0.265, -0.265)
        private val BACK_LEFT_LOC = Translation2d(-0.265, 0.265)
        private val BACK_RIGHT_LOC = Translation2d(-0.265, -0.265)
        val SWERVE_KINEMATICS =
            SwerveDriveKinematics(FRONT_LEFT_LOC, FRONT_RIGHT_LOC, BACK_LEFT_LOC, BACK_RIGHT_LOC)

        val WHEEL_ENCODER_OFFSETS =
            mapOf(
                WheelLocation.FRONT_LEFT to FRONT_LEFT_ENCODER_OFFSET,
                WheelLocation.FRONT_RIGHT to FRONT_RIGHT_ENCODER_OFFSET,
                WheelLocation.BACK_LEFT to BACK_LEFT_ENCODER_OFFSET,
                WheelLocation.BACK_RIGHT to BACK_RIGHT_ENCODER_OFFSET
            )
    }

    object WheelConstants {
        const val ROTATIONS_TO_METERS = 0.245 / 5.08
    }

    object ArmConstants {
        const val PIVOT_MOTOR_ID = 10
        const val TELESCOPE_MOTOR_ID = 11
        const val pivotOffset = 0.687
        const val startingPivotG = 0.8 // 2.01;
        const val pivotGPerTelescopeMeter = 0.8
        const val telescopeRotationToMeters = -0.09 / 32768
        const val telescopeOuterSetpoint = 0.9
        const val telescopeMiddleSetpoint = 0.3
        const val telescopeBottomSetpoint = 0.3
        const val telescopeSubstationSetpoint = 0.568
        const val elevatorUpPivotDownPosition = 0.81
        const val pivotDownPosition = 0.83
        const val topConePosition = 0.015
        const val middleConePosition = 0.015
        const val bottomConePosition = 0.015
        const val substationConePosition = 0.015
    }

    object GrabberConstants {
        const val PNEUMATICS_HUB_ID = 23
        const val LEFT_SOLENOID_REVERSE = 3
        const val LEFT_SOLENOID_FORWARD = 2
        const val RIGHT_SOLENOID_REVERSE = 1
        const val RIGHT_SOLENOID_FORWARD = 0
    }

    object ElevatorConstants {
        const val ELEVATOR_MOTOR_ID = 9
    }

    object DriverConstants {
        const val speedMultiplier = 3.0
        const val angleMultiplier = 3.0
    }

    object AutoConstants {
        const val autoTelescopeIn = 0.45
    }

    object LedConstants {
        const val BLINKIN_CHANNEL = 0
    }

    enum class ConePosition {
        TOP,
        MIDDLE,
        BOTTOM,
        SUBSTATION
    }
}
