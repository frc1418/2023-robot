package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class TelescopeSubsystem extends SubsystemBase {

  private final TalonFX telescopeMotor;

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/arm");

  private final NetworkTableEntry ntTelescopeLength = table.getEntry("telescopeLength");

  public TelescopeSubsystem(TalonFX telescopeMotor) {
    this.telescopeMotor = telescopeMotor;

    this.telescopeMotor.selectProfileSlot(0, 0);
    this.telescopeMotor.config_kP(0, 1.2);
    this.telescopeMotor.config_kI(0, 0);
    this.telescopeMotor.config_kD(0, 0.5);
    this.telescopeMotor.config_kF(0, 0);

    this.telescopeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    this.telescopeMotor.setSensorPhase(true);
    this.telescopeMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  @Override
  public void periodic() {
    updateTelescopeLength();
  }

  private void updateTelescopeLength() {
    ntTelescopeLength.setDouble(
        ArmConstants.telescopeRotationToMeters
            * telescopeMotor.getSensorCollection().getIntegratedSensorPosition());
  }

  private void setTelescopeTarget(double target) {
    telescopeMotor.set(ControlMode.Position, target);
  }

  public Command moveToTargetAndHold(double target) {
    return runOnce(() -> setTelescopeTarget(target)).andThen(Commands.waitUntil(this::atTarget));
  }

  public Command moveToTarget(double target) {
    return runOnce(() -> setTelescopeTarget(target));
  }

  public boolean atTarget() {
    double CLOSED_LOOP_THRESHOLD = 5;
    return Math.abs(telescopeMotor.getClosedLoopError()) < CLOSED_LOOP_THRESHOLD;
  }
}
