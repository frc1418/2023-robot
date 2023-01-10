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

        pidController.setOutputRange(-1, 1);
    }

    public void drive (SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
            new Rotation2d(turningEncoder.getDistance()));
        
        double speed = state.speedMetersPerSecond;
        Rotation2d angle = optimizedState.angle;

        speedMotor.set(speed / Math.sqrt(2));
        pidController.setReference(angle.getRadians(), ControlType.kPosition);
    }

    public CANSparkMax getAngleMotor(){
        return angleMotor;
    }

    public CANSparkMax getSpeedMotor(){
        return speedMotor;
    }
}
