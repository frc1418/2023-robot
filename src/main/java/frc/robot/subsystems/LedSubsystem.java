// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedColor;

public class LedSubsystem extends SubsystemBase {

  public LedColor color;

  public LedSubsystem(Spark blinkin) {

    setAllianceColor();

    setDefaultCommand(runOnce(() -> blinkin.set(color.color())));
  }

  public void setAllianceColor() {
    if (DriverStation.getAlliance() == Alliance.Blue) color = LedColor.BLUE_ALLIANCE;
    else color = LedColor.RED_ALLIANCE;
  }

  public Command showBalancingColorCommand() {
    return runOnce(() -> color = LedColor.BALANCING);
  }

  public Command showDockedColorCommand() {
    return runOnce(() -> color = LedColor.DOCKED);
  }

  public Command showGrabberOpenCommand() {
    return runOnce(() -> color = LedColor.GRABBER_OPEN);
  }

  public Command showGrabberClosedCommand() {
    return runOnce(() -> color = LedColor.GRABBER_CLOSED);
  }
}
