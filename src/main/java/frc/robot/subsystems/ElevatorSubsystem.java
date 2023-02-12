package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    DigitalInput topLimitSwitch = new DigitalInput(3);
    // DigitalInput bottomLimitSwitch = new DigitalInput(0);

    private CANSparkMax elevatorMotor;
    public ElevatorSubsystem(CANSparkMax elevatorMotor) {
        this.elevatorMotor = elevatorMotor;
    }

    public void setElevatorMotor(double speed) {
        if (topLimitSwitch.get() && speed < 0)
            elevatorMotor.set(0);
        else
            elevatorMotor.set(speed);
    }
    
}
