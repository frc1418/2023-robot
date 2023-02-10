package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor;
    private TalonFX telescopeMotor;
    private DutyCycleEncoder pivotEncoder;
    public ArmSubsystem(CANSparkMax pivotMotor, TalonFX telescopeMotor) {
        this.pivotMotor = pivotMotor;
        this.telescopeMotor = telescopeMotor;
        this.pivotEncoder = new DutyCycleEncoder(0);
    }

    public void setPivotMotor(double speed) {
        if (speed != 0)
            System.out.println(pivotEncoder.get());
        pivotMotor.setVoltage(speed);
    }

    public void setTelescopeMotor(double speed){
        telescopeMotor.set(ControlMode.Current, speed);
    }
    
}