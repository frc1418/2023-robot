package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightDirections;

public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private final NetworkTableEntry ntRobotposeTargetspace = table.getEntry("botpose_targetspace");
  private final NetworkTableEntry ntCameraposeTargetspace =
      table.getEntry("camerapose_targetspace");

  // returns 0 if no target, 1, if target
  private final NetworkTableEntry ntIsDetecting = table.getEntry("tv");

  private final NetworkTableEntry ntID = table.getEntry("tid");

  private double xToTarget;
  private double yToTarget;
  private LimelightDirections targetRotation;

  public LimelightSubsystem() {}

  @Override
  public void periodic() {

    double[] botPosArray = ntRobotposeTargetspace.getDoubleArray(new double[6]);
    double[] camPosArray = ntCameraposeTargetspace.getDoubleArray(new double[6]);

    xToTarget = botPosArray[0];
    yToTarget = botPosArray[2];
    double rotToTarget = camPosArray[4];

    double ntIdValue = ntID.getDouble(0);

    if (DriverStation.getAlliance() == Alliance.Blue) {
      if (ntIdValue == 6 || ntIdValue == 7 || ntIdValue == 8) {
        targetRotation = LimelightDirections.GRID_SIDE;
      } else if (ntIdValue == 4) {
        targetRotation = LimelightDirections.SUBSTATION_SIDE;
      }
    } else {
      if (ntIdValue == 3 || ntIdValue == 2 || ntIdValue == 1) {
        targetRotation = LimelightDirections.GRID_SIDE;
      } else if (ntID.getDouble(0) == 5) {
        targetRotation = LimelightDirections.SUBSTATION_SIDE;
      }
    }

    double distanceToTarget = Math.hypot(xToTarget, yToTarget);
    Rotation2d angleToTarget = Rotation2d.fromRadians(Math.atan2(xToTarget, yToTarget));
  }

  public double getXDistance() {
    return xToTarget;
  }

  public double getYDistance() {
    return yToTarget;
  }

  public boolean getIsDetecting() {
    int id = (int) ntID.getInteger(Integer.MAX_VALUE);
    return ntIsDetecting.getInteger(0) == 1 && id >= 1 && id <= 8;
  }

  public LimelightDirections getTargetRotation() {
    return targetRotation;
  }
}
