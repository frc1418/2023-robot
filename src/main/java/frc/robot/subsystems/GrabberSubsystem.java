package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

  private final DoubleSolenoid leftSolenoid;
  private final DoubleSolenoid rightSolenoid;

  public GrabberSubsystem(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {
    this.leftSolenoid = leftSolenoid;
    this.rightSolenoid = rightSolenoid;
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  public Command release() {
    return runOnce(this::openGrabber).andThen(Commands.waitSeconds(0.2));
  }

  public Command grab() {
    return runOnce(this::closeGrabber).andThen(Commands.waitSeconds(0.2));
  }

  public Command toggle() {
    return runOnce(this::toggleGrabber).andThen(Commands.waitSeconds(0.2));
  }

  private void closeGrabber() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  private void openGrabber() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  private void toggleGrabber() {
    leftSolenoid.toggle();
    rightSolenoid.toggle();
  }

  public boolean isOpen() {
    return leftSolenoid.get() == Value.kForward;
  }
}
