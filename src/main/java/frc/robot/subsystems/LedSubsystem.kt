// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.LedColor

class LedSubsystem(blinkin: Spark) : SubsystemBase() {
    private var color: LedColor = LedColor.BLUE_ALLIANCE

    init {
        setAllianceColor()
        defaultCommand = runOnce { blinkin.set(color.color) }
    }

    private fun setAllianceColor() {
        color =
            if (DriverStation.getAlliance() == Alliance.Blue) LedColor.BLUE_ALLIANCE
            else LedColor.RED_ALLIANCE
    }

    fun showBalancingColorCommand(): Command {
        return runOnce { color = LedColor.BALANCING }
    }

    fun showDockedColorCommand(): Command {
        return runOnce { color = LedColor.DOCKED }
    }

    fun showGrabberOpenCommand(): Command {
        return runOnce { color = LedColor.GRABBER_OPEN }
    }

    fun showGrabberClosedCommand(): Command {
        return runOnce { color = LedColor.GRABBER_CLOSED }
    }
}
