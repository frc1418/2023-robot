package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.WheelConstants;

public class WheelSubsystem {

  private final SparkMaxPIDController anglePIDController;
  private final SparkMaxPIDController speedPIDController;
  private final SparkMaxAbsoluteEncoder turningEncoder;
  private final RelativeEncoder speedEncoder;
  private final NetworkTableEntry ntSpeedTarget;
  private final NetworkTableEntry ntVelocity;
  private final NetworkTableEntry angleEncoderNtEntry;

  private double wheelSpeedSetpoint = 0.0;

  public WheelSubsystem(CANSparkMax angleMotor, CANSparkMax speedMotor, String ntName) {
    this.speedEncoder = speedMotor.getEncoder();
    this.turningEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    NetworkTable table = ntInstance.getTable("/components/drivetrain/" + ntName);
    this.angleEncoderNtEntry = table.getEntry("AngleEncoder");
    this.ntSpeedTarget = table.getEntry("speedTarget");
    this.ntVelocity = table.getEntry("wheelVelocity");
    this.ntSpeedTarget.setDouble(0);
    this.ntVelocity.setDouble(0);

    this.speedEncoder.setPosition(0);
    this.speedEncoder.setPositionConversionFactor(WheelConstants.ROTATIONS_TO_METERS);
    this.speedEncoder.setVelocityConversionFactor(
        this.speedEncoder.getPositionConversionFactor() / 60.0);

    this.speedPIDController = speedMotor.getPIDController();
    speedPIDController.setP(0.0001);
    speedPIDController.setI(0.00);
    speedPIDController.setD(0.00);
    speedPIDController.setFF(0.25);

    this.anglePIDController = angleMotor.getPIDController();
    anglePIDController.setPositionPIDWrappingMinInput(1);
    anglePIDController.setPositionPIDWrappingMaxInput(1);
    anglePIDController.setOutputRange(-1, 1);
  }

  public void drive(SwerveModuleState state) {
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(state, Rotation2d.fromRotations(turningEncoder.getPosition()));

    wheelSpeedSetpoint = optimizedState.speedMetersPerSecond;
    speedPIDController.setReference(wheelSpeedSetpoint, ControlType.kVelocity);

    Rotation2d angle = optimizedState.angle;
    anglePIDController.setReference(angle.getRotations(), ControlType.kPosition);
  }

  public SparkMaxAbsoluteEncoder getEncoder() {
    return turningEncoder;
  }

  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(
        speedEncoder.getPosition(), Rotation2d.fromRotations(turningEncoder.getPosition()));
  }

  public void periodic() {
    angleEncoderNtEntry.setDouble(turningEncoder.getPosition());
    ntSpeedTarget.setDouble(wheelSpeedSetpoint);
    ntVelocity.setDouble(speedEncoder.getVelocity());
  }
}
