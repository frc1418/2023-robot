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

    private double targetVoltage = 0;

    private double angleSetpoint = 0;


    public WheelSubsystem (CANSparkMax angleMotor, CANSparkMax speedMotor, AnalogEncoder turningEncoder, Translation2d location) {
        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.location = location;
        this.turningEncoder = turningEncoder;

        pidController = new PIDController(4, 0, 0);
        pidController.enableContinuousInput(0, 1);
        pidController.setTolerance(1.0/360);
    }

    public void drive (SwerveModuleState state) {
        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
            Rotation2d.fromRotations(getEncoderPosition()));
            
        // SwerveModuleState returns a speed between 0 and sqrt(2)
        // for x and y values between 0 and 1. Dividing by sqrt(2)
        // converts the range to 0-1.
        targetVoltage = optimizedState.speedMetersPerSecond;
        speedMotor.set(targetVoltage / Math.sqrt(2));

        Rotation2d angle = optimizedState.angle;
        System.out.println(angle.getRotations());
        double pidOutput = pidController.calculate(getEncoderPosition(), angle.getRotations());
        double clampedPidOutpt = MathUtil.clamp(pidOutput, -1, 1);
        
        if (!pidController.atSetpoint())
            angleSetpoint = clampedPidOutpt;
        else
            angleSetpoint = 0;

        angleMotor.set(angleSetpoint);
        
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
        double rawPos = turningEncoder.getAbsolutePosition() - turningEncoder.getPositionOffset();
        if (rawPos < 0)
            return 1 + rawPos;
        else
            return rawPos;
    }

    public double getangleSetpoint() {
        return angleSetpoint;
    }
}
