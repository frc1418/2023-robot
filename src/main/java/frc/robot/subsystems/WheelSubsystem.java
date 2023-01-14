package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
    private PIDController pidController;
    private AnalogEncoder turningEncoder;    
    
    Translation2d location;
    private final double MAX_VOLTS = 4.95;

    private double targetVoltage = 0;

    private boolean invertEncoder;

    public WheelSubsystem (CANSparkMax angleMotor, CANSparkMax speedMotor, AnalogEncoder turningEncoder, Translation2d location, boolean invertEncoder) {
        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.location = location;
        this.turningEncoder = turningEncoder;
        this.invertEncoder = invertEncoder;

        pidController = new PIDController(0.06, 0, 0);
    }

    public void drive (SwerveModuleState state) {
        //this.turningEncoder.reset();

        double encoderPos;

        if(invertEncoder){
            encoderPos = -turningEncoder.getAbsolutePosition();
        } else {
            encoderPos = turningEncoder.getAbsolutePosition();
        }

        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
            new Rotation2d(encoderPos));
            
        targetVoltage = optimizedState.speedMetersPerSecond;
        speedMotor.set(targetVoltage / Math.sqrt(2));

        Rotation2d angle = optimizedState.angle;
        double pidOutput = pidController.calculate(getEncoderPosition(), angle.getDegrees() / 360);
        double clampedPidOutpt = MathUtil.clamp(pidOutput, -1, 1);
        
        angleMotor.set(clampedPidOutpt);
        
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

    public double getEncoderPosition() {
        double rawPos = turningEncoder.getAbsolutePosition();// - turningEncoder.getPositionOffset();
        if (rawPos < 0)
            return 1 + rawPos;
        else
            return rawPos;
    }
}
