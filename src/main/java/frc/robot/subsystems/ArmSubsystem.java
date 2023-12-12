package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor;
    private TalonFX telescopeMotor;
    private SparkMaxAbsoluteEncoder pivotEncoder;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/arm");

    private final NetworkTableEntry ntPivotPosition = table.getEntry("pivotPosition");
    private final NetworkTableEntry ntTelescopeLength = table.getEntry("telescopeLength");

    private PIDController pivotPidController = new PIDController(19, 3, 0);//new PIDController(18, 0, 0);
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ArmConstants.startingPivotG, 0);

    public ArmSubsystem(CANSparkMax pivotMotor, TalonFX telescopeMotor) {
        this.pivotMotor = pivotMotor;
        this.telescopeMotor = telescopeMotor;

        this.telescopeMotor.selectProfileSlot(0, 0);
        this.telescopeMotor.config_kP(0, 1);
        this.telescopeMotor.config_kI(0, 0);
        this.telescopeMotor.config_kD(0, 0);
        this.telescopeMotor.config_kF(0, 0);
        
        this.pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.pivotEncoder.setZeroOffset(ArmConstants.pivotOffset);

        pivotPidController.enableContinuousInput(0, 1);

        this.telescopeMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    }

    // public void setPivotMotorVoltage(double speed) {
        
    //     pivotMotor.setVoltage(speed);
    // }

    // public void setPivotMotor(double speed) {
    //     pivotMotor.set(speed);
    // }

    // public void setTelescopeMotor(double speed){
    //     // System.out.println(speed);
    //     telescopeMotor.set(ControlMode.PercentOutput, speed);
    // }

    // public void setTelescopePosition(double pos) {
    //     // System.out.println(pos);
    //     // telescopeMotor.set(ControlMode.Position, -0.5/ArmConstants.telescopeRotationToMeters);
    //     telescopeMotor.set(ControlMode.Position, -pos /ArmConstants.telescopeRotationToMeters);
    // }

    // public void setPivotPosition(double pos) {
    //     System.out.println(ntPivotPosition.getDouble(0));
    //     pivotMotor.setVoltage(armFeedforward.calculate(pos, 0) + pivotPidController.calculate(ntPivotPosition.getDouble(0), pos));
    // }

    // @Override
    // public void periodic() {
    //     updatePivotPosition();
    //     updateTelescopeLength();
    // }

    // public void updatePivotPosition() {
    //     ntPivotPosition.setDouble(pivotEncoder.getPosition());
    // }

    // public void updateTelescopeLength() {
    //     ntTelescopeLength.setDouble(ArmConstants.telescopeRotationToMeters * telescopeMotor.getSensorCollection().getIntegratedSensorPosition());
    //     armFeedforward = new ArmFeedforward(0, ArmConstants.startingPivotG + ArmConstants.pivotGPerTelescopeMeter*ntTelescopeLength.getDouble(0), 0);
    // }

    // public void resetTelescopeEncoder() {
    //     telescopeMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    //     ntTelescopeLength.setDouble(0);
    // }

    // public double getPivotPosition() {
    //     return ntPivotPosition.getDouble(0);
    // }
    
}