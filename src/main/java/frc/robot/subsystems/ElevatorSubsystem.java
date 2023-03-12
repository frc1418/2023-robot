package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable ntArm = nt.getTable("/components/arm");
    NetworkTableEntry ntArmAngle = nt.getEntry("pivotPosition");
    NetworkTableEntry ntArmLength = nt.getEntry("telescopeLength");


    DigitalInput topLimitSwitch = new DigitalInput(3);
    DigitalInput bottomLimitSwitch = new DigitalInput(4);
    
    private TalonFX elevatorMotor;


    public ElevatorSubsystem(TalonFX elevatorMotor) {
        this.elevatorMotor = elevatorMotor;

        this.elevatorMotor.selectProfileSlot(0, 0);
        this.elevatorMotor.config_kP(0, 0);
        this.elevatorMotor.config_kI(0, 0);
        this.elevatorMotor.config_kD(0, 0);
        this.elevatorMotor.config_kF(0, 0);
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
        {
            System.out.println("MOVING");
            elevatorMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public void setElevatorHeight(double pos) {

        double offset = Math.cos(2*Math.PI*ntArmAngle.getDouble(0)) * ntArmLength.getDouble(0);

        elevatorMotor.set(ControlMode.Position, pos - offset);


    }
    
}
