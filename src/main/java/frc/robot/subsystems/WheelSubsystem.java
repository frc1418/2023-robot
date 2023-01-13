package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainSubsystem;

public class WheelSubsystem extends SubsystemBase{

    
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController pidController;
    private AnalogEncoder turningEncoder;    
    
    Translation2d location;
    private final double MAX_VOLTS = 4.95;

    private double targetVoltage = 0;

    public WheelSubsystem (CANSparkMax angleMotor, CANSparkMax speedMotor, int turningEncoder, Translation2d location) {
        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.location = location;
        this.turningEncoder = new AnalogEncoder(turningEncoder);
        
        pidController = this.angleMotor.getPIDController();

        

        // I imagine you'll want this to be much larger (~50)
        // When the error is 1/4 rotation, (90 degrees), the motor should probably run at 12v, so that means
        // kP * .25 = 12
        // kP       = 48
        pidController.setP(0.06);
        pidController.setI(0);
        pidController.setD(0);
        pidController.setFF(0.03);
        
        pidController.setPositionPIDWrappingEnabled(true);
        // The PID output should go from -12 to 12, since that's the possible values you can give the motor
        // -2*PI<->2*PI is really an input range
        pidController.setOutputRange(-2 * Math.PI, 2 * Math.PI);
    }

    public void drive (SwerveModuleState state) {
        //this.turningEncoder.reset();
        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
            new Rotation2d(turningEncoder.getDistance()));
        
        targetVoltage = optimizedState.speedMetersPerSecond;
        Rotation2d angle = optimizedState.angle;
        //if (location != DrivetrainSubsystem.m_frontRightLocation) {
            // Why not use the velocity control, like you use the position control?
            // I would set up a PIDController for the speed motor too
            speedMotor.set(targetVoltage / Math.sqrt(2));
            // Read the docs for `setReference`. Position should be in rotations, not radians.
            // I think you need to divide by 2PI.
            pidController.setReference(angle.getRadians(), ControlType.kPosition);
            //System.out.println("hi");
        //}
        
    }

    public CANSparkMax getAngleMotor(){
        return angleMotor;
    }

    public CANSparkMax getSpeedMotor(){
        return speedMotor;
    }

    public AnalogEncoder getEncoder(){
        return turningEncoder;
    }
    public double getTargetVoltage() {
        return targetVoltage;
    }
}
