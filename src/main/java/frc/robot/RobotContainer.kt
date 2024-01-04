// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.PathConstraints
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.ArmConstants
import frc.robot.Constants.ConePosition
import frc.robot.Constants.DriverConstants
import frc.robot.Constants.DrivetrainConstants
import frc.robot.Constants.ElevatorConstants
import frc.robot.Constants.GrabberConstants
import frc.robot.Constants.LedConstants
import frc.robot.commands.AlignByAprilTag
import frc.robot.commands.FollowTrajectoryCommand
import frc.robot.commands.LevelChargingStationCommand
import frc.robot.common.Odometry
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.GrabberSubsystem
import frc.robot.subsystems.LedSubsystem
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.PivotSubsystem
import frc.robot.subsystems.SwerveDriveSubsystem
import frc.robot.subsystems.TelescopeSubsystem
import frc.robot.subsystems.WheelSubsystem
import kotlin.math.abs

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer( // The robot's subsystems and commands are defined here...
) {
    // Front Left Module
    private val frontLeftSpeedMotor =
        CANSparkMax(
            DrivetrainConstants.FRONT_LEFT_SPEED_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless,
        )
    private val frontLeftAngleMotor =
        CANSparkMax(
            DrivetrainConstants.FRONT_LEFT_ANGLE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
    private val frontLeftWheel =
        WheelSubsystem(
            frontLeftAngleMotor,
            frontLeftSpeedMotor,
            "frontLeft",
        )

    // Front Right Module
    private val frontRightAngleMotor =
        CANSparkMax(
            DrivetrainConstants.FRONT_RIGHT_ANGLE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
    private val frontRightSpeedMotor =
        CANSparkMax(
            DrivetrainConstants.FRONT_RIGHT_SPEED_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
    private val frontRightWheel =
        WheelSubsystem(frontRightAngleMotor, frontRightSpeedMotor, "frontRight")

    // Back Left Module
    private val backLeftAngleMotor =
        CANSparkMax(
            DrivetrainConstants.BACK_LEFT_ANGLE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
    private val backLeftSpeedMotor =
        CANSparkMax(
            DrivetrainConstants.BACK_LEFT_SPEED_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
    private val backLeftWheel = WheelSubsystem(backLeftAngleMotor, backLeftSpeedMotor, "backLeft")

    // Back Right Module
    private val backRightAngleMotor =
        CANSparkMax(
            DrivetrainConstants.BACK_RIGHT_ANGLE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
    private val backRightSpeedMotor =
        CANSparkMax(
            DrivetrainConstants.BACK_RIGHT_SPEED_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
    private val backRightWheel =
        WheelSubsystem(backRightAngleMotor, backRightSpeedMotor, "backRight")

    private val ntInstance = NetworkTableInstance.getDefault()
    private val table = ntInstance.getTable("/components/Odometry")
    private val inclineAngle = table.getEntry("inclineAngle")
    private val inclineDirection = table.getEntry("inclineDirection")
    private val pitch = table.getEntry("pitch")
    private val roll = table.getEntry("roll")
    private val limelight = LimelightSubsystem()
    private val blinkin = Spark(LedConstants.BLINKIN_CHANNEL)
    val ledSubsystem = LedSubsystem(blinkin)
    private val pivotMotor =
        CANSparkMax(ArmConstants.PIVOT_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val telescopeMotor = TalonFX(ArmConstants.TELESCOPE_MOTOR_ID)
    private val pivotSubsystem = PivotSubsystem(pivotMotor, telescopeMotor.sensorCollection)
    private val telescopeSubsystem = TelescopeSubsystem(telescopeMotor)
    private val leftSolenoid =
        DoubleSolenoid(
            GrabberConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH,
            GrabberConstants.LEFT_SOLENOID_FORWARD,
            GrabberConstants.LEFT_SOLENOID_REVERSE
        )
    private val rightSolenoid =
        DoubleSolenoid(
            GrabberConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH,
            GrabberConstants.RIGHT_SOLENOID_FORWARD,
            GrabberConstants.RIGHT_SOLENOID_REVERSE
        )
    private val grabberSubsystem = GrabberSubsystem(leftSolenoid, rightSolenoid)
    private val elevatorMotor =
        CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val elevatorSubsystem = ElevatorSubsystem(elevatorMotor)
    private val eventMap = HashMap<String, Command?>()

    // SENDABLE CHOOSER
    private val chooser = SendableChooser<Command>()
    private val positions =
        arrayOf(
            frontLeftWheel.swerveModulePosition,
            frontRightWheel.swerveModulePosition,
            backLeftWheel.swerveModulePosition,
            backRightWheel.swerveModulePosition
        )
    private val gyro = AHRS(SPI.Port.kMXP)
    private val driveOdometry =
        SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, gyro.rotation2d, positions)
    val odometry = Odometry(gyro, driveOdometry, positions, limelight)
    val swerveDriveSubsystem =
        SwerveDriveSubsystem(
            backRightWheel,
            backLeftWheel,
            frontRightWheel,
            frontLeftWheel,
            DrivetrainConstants.SWERVE_KINEMATICS,
            odometry
        )
    private val levelChargingStationCommand: Command =
        ledSubsystem
            .showBalancingColorCommand()
            .andThen(
                LevelChargingStationCommand(odometry, swerveDriveSubsystem),
                ledSubsystem.showDockedColorCommand()
            )
    private val alignAtAprilTag =
        AlignByAprilTag(swerveDriveSubsystem, limelight, odometry, 0.0, -0.77) // 1.1);
    private val alignLeftOfAprilTag =
        AlignByAprilTag(swerveDriveSubsystem, limelight, odometry, 0.64, -0.77)
    private val alignRightOfAprilTag =
        AlignByAprilTag(swerveDriveSubsystem, limelight, odometry, -0.64, -0.77)
    private val limitX = SlewRateLimiter(6.0)
    private val limitY = SlewRateLimiter(6.0)

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private fun followTrajectory(name: String?, constraints: PathConstraints?): Command {
        return FollowTrajectoryCommand(name, odometry, swerveDriveSubsystem, eventMap, constraints)
    }

    private fun deliverCone(conePosition: ConePosition): Command {
        return grabberSubsystem.grab.andThen(
            pivotSubsystem
                .setTarget(getArmPositionForCone(conePosition))
                .andThen(
                    telescopeSubsystem
                        .setTargetAndWait(getTelescopePositionForCone(conePosition))
                        .andThen(grabberSubsystem.release, telescopeSubsystem.setTarget(0.03))
                ),
            pivotSubsystem.setTarget(ArmConstants.elevatorUpPivotDownPosition)
        )
    }

    init {
        val balance =
            swerveDriveSubsystem.disableFieldCentricCommand.andThen(
                PrintCommand("LEVELING"),
                ledSubsystem.showBalancingColorCommand(),
                LevelChargingStationCommand(odometry, swerveDriveSubsystem),
                ledSubsystem.showDockedColorCommand()
            )
        val middleToChargingStationTrajectory =
            followTrajectory("middleToChargingStation", PathConstraints(4.0, 4.0))
        // Move robot from the middle position onto the charging station and balances
        val middleToChargingStation =
            pivotSubsystem
                .setTarget(ArmConstants.pivotDownPosition)
                .andThen(
                    telescopeSubsystem
                        .setTarget(0.03)
                        .andThen(
                            middleToChargingStationTrajectory,
                            balance.alongWith(elevatorSubsystem.lowerElevatorCommand())
                        )
                )
        val chargeCommand = followTrajectory("middleToChargingStation", PathConstraints(2.5, 2.5))
        val leftToLeftBallBlue = followTrajectory("leftToLeftBallBlue", PathConstraints(2.5, 2.5))
        val leftToLeftBallRed = followTrajectory("leftToLeftBallRed", PathConstraints(2.5, 2.5))
        val retrieveLeftBall =
            Commands.either(leftToLeftBallRed, leftToLeftBallBlue) {
                    DriverStation.getAlliance() == Alliance.Red
                }
                .andThen(
                    swerveDriveSubsystem.stop(),
                    grabberSubsystem.release,
                    telescopeSubsystem.setTarget(0.03)
                )
        val rightBack = followTrajectory("rightBack", PathConstraints(1.72, 2.5))
        val deliverUpperCone = deliverCone(ConePosition.TOP)
        val deliverMiddleCone = deliverCone(ConePosition.MIDDLE)
        val leftUpperConeAutonomous = deliverUpperCone.andThen(retrieveLeftBall)
        val rightUpperConeAutonomous = deliverUpperCone.andThen(rightBack)
        val middleAutonomous: Command = deliverUpperCone.andThen(middleToChargingStation)
        val middleAutonomousMiddleCone = deliverMiddleCone.andThen(middleToChargingStation)
        val upperConeAutonomous = deliverUpperCone.andThen(elevatorSubsystem.lowerElevatorCommand())
        chooser.addOption("Charge Command", chargeCommand)
        chooser.addOption("Left Upper Cone Autonomous", leftUpperConeAutonomous)
        chooser.addOption("Right Upper Cone Autonomous", rightUpperConeAutonomous)
        chooser.addOption("Middle Autonomous Middle Cone", middleAutonomousMiddleCone)
        chooser.addOption("Middle Autonomous Upper Cone", middleAutonomous)
        chooser.addOption("Middle Autonomous No Cone", middleToChargingStation)
        chooser.addOption("Just Upper Cone Autonomous :(", upperConeAutonomous)
        chooser.setDefaultOption("Middle Autonomous", middleAutonomous)
        SmartDashboard.putData(chooser)
        inclineAngle.setDefaultDouble(0.0)
        inclineDirection.setDefaultDouble(0.0)
        pitch.setDefaultDouble(0.0)
        roll.setDefaultDouble(0.0)

        // Configure the button bindings
        configureButtonBindings()
        configureObjects()
        buildAutoEventMap()
    }

    private fun configureObjects() {
        frontLeftSpeedMotor.inverted = true
        frontLeftAngleMotor.inverted = false

        frontRightSpeedMotor.inverted = false
        frontRightAngleMotor.inverted = false

        backRightSpeedMotor.inverted = true
        backRightAngleMotor.inverted = false

        backLeftSpeedMotor.inverted = false
        backLeftAngleMotor.inverted = false

        backRightAngleMotor.absoluteEncoder.setZeroOffset(
            DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET
        )
        backLeftAngleMotor.absoluteEncoder.setZeroOffset(
            DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET
        )
        frontRightAngleMotor.absoluteEncoder.setZeroOffset(
            DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET
        )
        frontLeftAngleMotor.absoluteEncoder.setZeroOffset(
            DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET
        )

        elevatorMotor.inverted = true
        telescopeMotor.inverted = true
        pivotMotor.setIdleMode(IdleMode.kBrake)
        elevatorMotor.setIdleMode(IdleMode.kBrake)
        telescopeMotor.setNeutralMode(NeutralMode.Brake)
        coastDrive()
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then
     * passing it to a [ ].
     */
    private fun configureButtonBindings() {
        val leftJoystick = Joystick(0)
        val rightJoystick = Joystick(1)
        val altJoystick = Joystick(2)
        val balanceChargingStationButton = JoystickButton(leftJoystick, 1)
        val turtleButton = JoystickButton(rightJoystick, 1)
        val fieldCentricButton = JoystickButton(rightJoystick, 2)
        val telescopeSubstation = JoystickButton(altJoystick, 8)
        val elevatorUpButton = JoystickButton(altJoystick, 6)
        val elevatorDownButton = JoystickButton(altJoystick, 5)
        val toggleGrabberButton = JoystickButton(altJoystick, 9)
        val pivotToTopPegButton = Trigger { altJoystick.pov == 0 }
        val pivotToBottomButton = Trigger { altJoystick.pov == 180 }
        val telescopeToOuterButton = Trigger { altJoystick.pov == 90 }
        val telescopeToMiddleButton = Trigger { altJoystick.pov == 270 }
        val telescopeToInButton = JoystickButton(altJoystick, 7)
        val alignRightOfAprilTagButton = JoystickButton(leftJoystick, 4)
        val alignAtAprilTagButton = JoystickButton(leftJoystick, 2)
        val alignLeftOfAprilTagButton = JoystickButton(leftJoystick, 3)
        val resetGyroButton = JoystickButton(rightJoystick, 8)

        swerveDriveSubsystem.defaultCommand =
            swerveDriveSubsystem.run {
                swerveDriveSubsystem.drive(
                    limitX.calculate(
                        applyDeadband(-leftJoystick.y, DrivetrainConstants.DRIFT_DEADBAND) *
                            DriverConstants.speedMultiplier
                    ),
                    limitY.calculate(
                        applyDeadband(-leftJoystick.x, DrivetrainConstants.DRIFT_DEADBAND) *
                            DriverConstants.speedMultiplier
                    ),
                    applyDeadband(
                        -rightJoystick.x * DriverConstants.angleMultiplier,
                        DrivetrainConstants.ROTATION_DEADBAND
                    )
                )
            }

        fieldCentricButton.onTrue(swerveDriveSubsystem.toggleFieldCentric())
        balanceChargingStationButton.whileTrue(
            ledSubsystem
                .showBalancingColorCommand()
                .andThen(levelChargingStationCommand, ledSubsystem.showDockedColorCommand())
        )
        turtleButton.whileTrue(swerveDriveSubsystem.turtleCommand)
        elevatorUpButton.whileTrue(elevatorSubsystem.raiseElevatorCommand())
        elevatorDownButton.whileTrue(elevatorSubsystem.lowerElevatorCommand())
        telescopeSubstation.onTrue(
            telescopeSubsystem.setTarget(ArmConstants.telescopeSubstationSetpoint)
        )

        Trigger(grabberSubsystem::isOpen)
            .onTrue(ledSubsystem.showGrabberOpenCommand())
            .onFalse(ledSubsystem.showGrabberClosedCommand())

        toggleGrabberButton.onTrue(grabberSubsystem.toggle)
        pivotToTopPegButton.onTrue(pivotSubsystem.setTarget(ArmConstants.topConePosition))
        pivotToBottomButton.onTrue(pivotSubsystem.setTarget(ArmConstants.bottomConePosition))
        telescopeToOuterButton.onTrue(
            telescopeSubsystem.setTarget(ArmConstants.telescopeOuterSetpoint)
        )
        telescopeToMiddleButton.onTrue(
            telescopeSubsystem.setTarget(ArmConstants.telescopeMiddleSetpoint)
        )
        telescopeToInButton.onTrue(telescopeSubsystem.setTarget(0.06))
        alignAtAprilTagButton.whileTrue(alignAtAprilTag)
        alignLeftOfAprilTagButton.whileTrue(alignLeftOfAprilTag)
        alignRightOfAprilTagButton.whileTrue(alignRightOfAprilTag)
        resetGyroButton.onTrue(
            InstantCommand(
                {
                    odometry.zeroHeading()
                    swerveDriveSubsystem.resetLockRot()
                    odometry.reset(
                        Pose2d(odometry.pose.x, odometry.pose.y, Rotation2d.fromDegrees(180.0))
                    )
                },
                swerveDriveSubsystem
            )
        )
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = chooser.selected

    private fun applyDeadband(input: Double, deadband: Double): Double {
        return if (abs(input) < deadband) 0.0 else input
    }

    fun coastDrive() {
        frontLeftAngleMotor.setIdleMode(IdleMode.kCoast)
        frontRightAngleMotor.setIdleMode(IdleMode.kCoast)
        backLeftAngleMotor.setIdleMode(IdleMode.kCoast)
        backRightAngleMotor.setIdleMode(IdleMode.kCoast)
        frontLeftSpeedMotor.setIdleMode(IdleMode.kCoast)
        frontRightSpeedMotor.setIdleMode(IdleMode.kCoast)
        backLeftSpeedMotor.setIdleMode(IdleMode.kCoast)
        backRightSpeedMotor.setIdleMode(IdleMode.kCoast)
    }

    private fun buildAutoEventMap() {
        eventMap["pickUpBall"] = grabberSubsystem.grab.andThen(pivotSubsystem.setTarget(0.995))
        eventMap["telescopeOut"] = telescopeSubsystem.setTarget(ArmConstants.telescopeOuterSetpoint)
    }

    private fun getArmPositionForCone(conePosition: ConePosition): Double {
        return when (conePosition) {
            ConePosition.TOP -> ArmConstants.topConePosition
            ConePosition.MIDDLE -> ArmConstants.middleConePosition
            ConePosition.BOTTOM -> ArmConstants.bottomConePosition
            ConePosition.SUBSTATION -> ArmConstants.substationConePosition
        }
    }

    private fun getTelescopePositionForCone(conePosition: ConePosition): Double {
        return when (conePosition) {
            ConePosition.TOP -> ArmConstants.telescopeOuterSetpoint
            ConePosition.MIDDLE -> ArmConstants.telescopeMiddleSetpoint
            ConePosition.BOTTOM -> ArmConstants.telescopeBottomSetpoint
            ConePosition.SUBSTATION -> ArmConstants.telescopeSubstationSetpoint
        }
    }
}

val CANSparkMax.absoluteEncoder: SparkMaxAbsoluteEncoder
    get() = this.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
