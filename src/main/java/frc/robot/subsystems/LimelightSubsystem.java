package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry robotpose_targetspace = table.getEntry("botpose_targetspace");

    private double distanceToTarget = 0;
    private double xToTarget;
    private double yToTarget;
    private double rotToTarget;

    private Rotation2d angleToTarget = new Rotation2d();
    
    
    public LimelightSubsystem() {

    }

    @Override
    public void periodic() {

        double[] posArray = robotpose_targetspace.getDoubleArray(new double[6]);

        xToTarget = posArray[0];
        yToTarget = posArray[2];
        rotToTarget = posArray[4];

        distanceToTarget = Math.hypot(xToTarget, yToTarget);
        angleToTarget = Rotation2d.fromRadians(Math.atan2(yToTarget, xToTarget));

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


    public double getRotation() {
        return rotToTarget;
    }

}
