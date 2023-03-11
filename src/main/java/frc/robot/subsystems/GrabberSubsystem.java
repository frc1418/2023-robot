package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;
    private boolean grabberClosed = false;
    public GrabberSubsystem(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);
    }

    public void toggle(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
        System.out.println(leftSolenoid.get());
        if (grabberClosed)
            grabberClosed = false;
        else
            grabberClosed = true;

    }

    public void grab(){
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kReverse);
        grabberClosed = true;
    }

    public void open() {
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);
        grabberClosed = false;
        
    }

    public boolean getGrabberClosed() {
        return grabberClosed;
    }
    
}
