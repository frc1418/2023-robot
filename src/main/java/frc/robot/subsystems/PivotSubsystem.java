package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class PivotSubsystem extends SubsystemBase {

  private final CANSparkMax pivotMotor;
  private final SparkMaxAbsoluteEncoder pivotEncoder;

  private final TalonFXSensorCollection telescopeSensor;

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/arm");

  private final NetworkTableEntry ntPivotPosition = table.getEntry("pivotPosition");

  private final PIDController pivotPidController;

  private double targetPos;

  public PivotSubsystem(CANSparkMax pivotMotor, TalonFXSensorCollection telescopeSensor) {
    this.pivotMotor = pivotMotor;
    this.telescopeSensor = telescopeSensor;

    this.pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    this.pivotEncoder.setZeroOffset(ArmConstants.pivotOffset);

    this.pivotPidController = new PIDController(19, 0, 3);
    this.pivotPidController.setTolerance(3, 1);
    this.pivotPidController.enableContinuousInput(0, 1);

    targetPos = pivotEncoder.getPosition();

    setDefaultCommand(moveToTargetContinuously());
  }

  @Override
  public void periodic() {
    ntPivotPosition.setDouble(pivotEncoder.getPosition());
  }

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  private ArmFeedforward getArmFeedforward() {
    double kG =
        ArmConstants.startingPivotG
            + ArmConstants.pivotGPerTelescopeMeter * telescopeSensor.getIntegratedSensorPosition();
    return new ArmFeedforward(0, kG, 1);
  }

  private void runMotor() {
    pivotMotor.set(
        getArmFeedforward().calculate(targetPos, 0)
            + pivotPidController.calculate(pivotEncoder.getPosition(), targetPos));
  }

  /**
   * Mainly used for command groups, where no default command is run
   *
   * @return A command that continuously runs the motor to the set target.
   */
  private Command moveToTargetContinuously() {
    return run(this::runMotor);
  }

  private Command moveToTargetUntilThere() {
    return run(this::runMotor).until(pivotPidController::atSetpoint);
  }

  /**
   * @param pos Position to move arm
   * @return A command that moves arm to pos and keeps it there. This command does not end
   */
  public Command moveToTargetContinuously(double pos) {
    return runOnce(() -> setTarget(pos)).andThen(moveToTargetContinuously());
  }

  /**
   * @param pos Position for the arm to move to
   * @return A command that moves the arm to the setpoint, finishing when the arm is at the setpoint
   */
  public Command moveToTargetUntilThere(double pos) {
    return runOnce(() -> setTarget(pos)).andThen(moveToTargetUntilThere());
  }

  /**
   * @param pos Position for arm to move to
   * @return A command that sets the position of the arm and ends, allowing the default command to
   *     control movement
   */
  public Command moveToTarget(double pos) {
    return runOnce(() -> setTarget(pos));
  }

  private void setTarget(double targetPos) {
    pivotPidController.reset();
    this.targetPos = targetPos;
  }
}
