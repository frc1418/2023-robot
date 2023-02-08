package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;
    public GrabberSubsystem(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;
    }

    public void toggle(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }
    
}
