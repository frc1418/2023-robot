package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry ntRobotposeTargetspace = table.getEntry("botpose_targetspace");
    private NetworkTableEntry ntCameraposeTargetspace = table.getEntry("camerapose_targetspace");

    // returns 0 if no target, 1, if target
    private NetworkTableEntry ntIsDetecting = table.getEntry("tv");

    private NetworkTableEntry ntID = table.getEntry("tid");

    private double distanceToTarget = 0;
    private double xToTarget;
    private double yToTarget;
    private double rotToTarget;
    private double targetRotation;

    private Rotation2d angleToTarget = new Rotation2d();
    
    
    public LimelightSubsystem() {

    }

    @Override
    public void periodic() {

        double[] botPosArray = ntRobotposeTargetspace.getDoubleArray(new double[6]);
        double [] camPosArray = ntCameraposeTargetspace.getDoubleArray(new double[6]);

        xToTarget = botPosArray[0];
        yToTarget = botPosArray[2];
        rotToTarget = camPosArray[4];

        if(DriverStation.getAlliance() == Alliance.Blue){
            if(ntID.getDouble(0) == 6 || ntID.getDouble(0) == 7 || ntID.getDouble(0) == 8){
                targetRotation = 180;
            } else if (ntID.getDouble(0) == 4){
                targetRotation = 0;
            }
        } else {
            if(ntID.getDouble(0) == 3 || ntID.getDouble(0) == 2 || ntID.getDouble(0) == 1){
                targetRotation = 180;
            } else if (ntID.getDouble(0) == 5){
                targetRotation = 0;
            }
        }

        distanceToTarget = Math.hypot(xToTarget, yToTarget);
        angleToTarget = Rotation2d.fromRadians(Math.atan2(xToTarget, yToTarget));

    }

    public double getDistance() {
        return distanceToTarget;
    }

    public double getXDistance() {
        return xToTarget;
    }

    public double getYDistance() {
        return yToTarget;
    }


    public Rotation2d getRotationToTargetPlane() {
        Rotation2d rot = Rotation2d.fromDegrees(rotToTarget - 90 + angleToTarget.getDegrees());
        // System.out.println(rotToTarget);
        return Rotation2d.fromDegrees(rotToTarget);
    }

    public boolean getIsDetecting() {
        int id = (int) ntID.getInteger(Integer.MAX_VALUE);
        boolean isDetecting = ntIsDetecting.getInteger(0) == 1 && id >= 1 && id <= 8;
        if (isDetecting && id != 6){
            // System.out.println("DETECTING " + id);
        }
        return isDetecting;
    }

    public double getTargetRotation() {
        return targetRotation;
    }

}
