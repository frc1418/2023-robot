// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    private val nt = NetworkTableInstance.getDefault()
    private val table = nt.getTable("robot")
    private val ntIsEnabled = table.getEntry("isEnabled")
    private lateinit var robotContainer: RobotContainer

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        ntIsEnabled.setBoolean(false)
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer()
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode. */
    override fun disabledInit() {
        ntIsEnabled.setBoolean(false)
        robotContainer.coastDrive()
        robotContainer.odometry.zeroHeading()
        robotContainer.odometry.setAngleOffset(180.0)
        robotContainer.odometry.reset(Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)))
        robotContainer.swerveDriveSubsystem.resetLockRot()
    }

    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class. */
    override fun autonomousInit() {
        ntIsEnabled.setBoolean(true)
        robotContainer.ledSubsystem.showAllianceColorCommand()

        // schedule the autonomous command (example)
        robotContainer.autonomousCommand.schedule()
    }

    /** This function is called periodically during autonomous. */
    override fun autonomousPeriodic() {}

    override fun autonomousExit() {
        super.autonomousExit()
        robotContainer.autonomousCommand.cancel()
    }

    override fun teleopInit() {
        ntIsEnabled.setBoolean(true)
        robotContainer.swerveDriveSubsystem.fieldCentric = true
        // robotContainer.configureObjects();
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }
}
