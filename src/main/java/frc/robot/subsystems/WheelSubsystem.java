package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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

    public WheelSubsystem (int angleMotor, int speedMotor, int turningEncoder, Translation2d location) {
        this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.location = location;
        this.turningEncoder = new AnalogEncoder(turningEncoder);
        pidController = this.angleMotor.getPIDController();

        pidController.setP(0.02);
        pidController.setI(0);
        pidController.setD(0);
        pidController.setFF(0.03);
        
        pidController.setPositionPIDWrappingEnabled(true);
        pidController.setOutputRange(-2 * Math.PI, 2 * Math.PI);
    }

    public void drive (SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
            new Rotation2d(turningEncoder.getDistance()));
        
        double speed = state.speedMetersPerSecond;
        Rotation2d angle = optimizedState.angle;
        //if (location != DrivetrainSubsystem.m_frontRightLocation) {
            speedMotor.set(speed / Math.sqrt(2));
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
}
