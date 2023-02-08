package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor;
    private Talon telescopeMotor;
    public ArmSubsystem(CANSparkMax pivotMotor, Talon telescopeMotor) {
        this.pivotMotor = pivotMotor;
        this.telescopeMotor = telescopeMotor;
    }

    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    public void setTelescopeMotor(double speed){
        telescopeMotor.set(speed);
    }
    
}