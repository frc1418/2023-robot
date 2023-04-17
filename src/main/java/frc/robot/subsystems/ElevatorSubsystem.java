package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax elevatorMotor;

  // TODO plug these limit switches into the spark max directly
  final DigitalInput topLimitSwitch = new DigitalInput(3);
  final DigitalInput bottomLimitSwitch = new DigitalInput(4);

  // TODO add encoder + hall effect sensor to elevator to allow for precise positioning
  public ElevatorSubsystem(CANSparkMax elevatorMotor) {
    this.elevatorMotor = elevatorMotor;
    setDefaultCommand(runOnce(() -> setElevatorMotor(0)));
  }

  private void setElevatorMotor(double speed) {
    elevatorMotor.set(speed);
  }

  // Prevent the motor from moving when the top limit switch is hit.
  // until() is only checked after running for 1 cycle, so we pair with unless()
  public Command raiseElevatorCommand() {
    return run(() -> setElevatorMotor(1)).until(topLimitSwitch::get).unless(topLimitSwitch::get);
  }

  public Command lowerElevatorCommand() {
    return run(() -> setElevatorMotor(-1))
        .until(bottomLimitSwitch::get)
        .unless(bottomLimitSwitch::get);
  }
}
