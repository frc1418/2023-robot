package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    DigitalInput topLimitSwitch = new DigitalInput(3);
    DigitalInput bottomLimitSwitch = new DigitalInput(4);
    
    private TalonFX elevatorMotor;
    public ElevatorSubsystem(TalonFX elevatorMotor) {
        this.elevatorMotor = elevatorMotor;
    }

    public void setElevatorMotor(double speed) {
        if (bottomLimitSwitch.get() && speed < 0)
        {
            System.out.println("NO MORE DOWN");
            elevatorMotor.set(ControlMode.PercentOutput, 0);
        }
        else if (topLimitSwitch.get() && speed > 0)
        {
            System.out.println("NO MORE UP");
            elevatorMotor.set(ControlMode.PercentOutput, 0);
        }
        else
            elevatorMotor.set(ControlMode.PercentOutput, speed);
    }
    
}
