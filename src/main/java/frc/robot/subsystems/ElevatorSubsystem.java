package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax elevatorMotor;
    public ElevatorSubsystem(CANSparkMax elevatorMotor) {
        this.elevatorMotor = elevatorMotor;
    }

    public void setElevatorMotor(double speed) {
        elevatorMotor.set(speed);
    }
    
}
