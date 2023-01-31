package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelSubsystem extends SubsystemBase{

    
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private PIDController anglePIDController;
    private SparkMaxPIDController speedPIDController;
    private AnalogEncoder turningEncoder;    
    
    Translation2d location;

    private double targetSpeed = 0;

    private double angleSetpoint = 0;
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry speedTarget = table.getEntry("speedTarget");
    private final NetworkTableEntry velocity = table.getEntry("wheelvelocity");

    public WheelSubsystem (CANSparkMax angleMotor, CANSparkMax speedMotor, AnalogEncoder turningEncoder, Translation2d location) {
        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.turningEncoder = turningEncoder;
        this.location = location;

        this.speedMotor.getEncoder().setPosition(0);
        this.speedMotor.getEncoder().setPositionConversionFactor(0.33/8.33);
        this.speedMotor.getEncoder().setVelocityConversionFactor(this.speedMotor.getEncoder().getPositionConversionFactor() / 60.0);
        this.speedPIDController = this.speedMotor.getPIDController();

        speedPIDController.setP(0.00);
        speedPIDController.setI(0.00);
        speedPIDController.setD(0.00);
        speedPIDController.setFF(0.259);
        



        anglePIDController = new PIDController(4, 0, 0);
        anglePIDController.enableContinuousInput(0, 1);
        anglePIDController.setTolerance(1.0/360);


        speedTarget.setDouble(0);
        velocity.setDouble(0);
    }

    public void drive (SwerveModuleState state) {
        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
            Rotation2d.fromRotations(getEncoderPosition()));
            
        targetSpeed = optimizedState.speedMetersPerSecond;
        speedPIDController.setReference(targetSpeed, ControlType.kVelocity);

        speedTarget.setDouble(targetSpeed);
        velocity.setDouble(speedMotor.getEncoder().getVelocity());

        Rotation2d angle = optimizedState.angle;
        double pidOutput = anglePIDController.calculate(getEncoderPosition(), angle.getRotations());
        double clampedPidOutpt = MathUtil.clamp(pidOutput, -1, 1);
        
        if (!anglePIDController.atSetpoint())
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
    public double gettargetSpeed() {
        return targetSpeed;
    }

    public double getEncoderPosition() {
        // return turningEncoder.getAbsolutePosition();
        double rawPos = turningEncoder.getAbsolutePosition() - turningEncoder.getPositionOffset();
        if (rawPos < 0)
            return -rawPos;
        else
            return 1 - rawPos;
    }

    public double getangleSetpoint() {
        return angleSetpoint;
    }

    public Translation2d getLocation() {
        return location;
    }

    public double getDistanceDriven() {
        return speedMotor.getEncoder().getPosition();
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            speedMotor.getEncoder().getPosition(), Rotation2d.fromRotations(getEncoderPosition()));
    }
}
