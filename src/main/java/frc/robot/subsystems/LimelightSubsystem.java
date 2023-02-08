package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry ntRobotposeTargetspace = table.getEntry("botpose_targetspace");

    // returns 0 if no target, 1, if target
    private NetworkTableEntry ntIsDetecting = table.getEntry("tv");

    private NetworkTableEntry ntID = table.getEntry("tid");

    private double distanceToTarget = 0;
    private double xToTarget;
    private double yToTarget;
    private double rotToTarget;

    private Rotation2d angleToTarget = new Rotation2d();
    
    
    public LimelightSubsystem() {

    }

    @Override
    public void periodic() {

        double[] posArray = ntRobotposeTargetspace.getDoubleArray(new double[6]);

        xToTarget = posArray[0];
        yToTarget = posArray[2];
        rotToTarget = posArray[4];

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
        System.out.println(rotToTarget);
        return rot;
    }

    public boolean getIsDetecting() {
        double id = ntID.getDouble(Integer.MAX_VALUE);
        boolean isDetecting = ntIsDetecting.getInteger(0) == 1 && ntID.getDouble(Integer.MAX_VALUE) >= 1 && ntID.getDouble(Integer.MAX_VALUE) <= 8;
        if (isDetecting){
            // System.out.println("DETECTING " + ntID.getDouble(1000));
        }
        return isDetecting;
    }

}
