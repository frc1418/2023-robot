package frc.robot.common;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Odometry {
    private final SwerveDriveOdometry odometry;
    private final AHRS gyro;
    private SwerveModulePosition[] modulePositions;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/Odometry");

    private final NetworkTableEntry inclineAngle = table.getEntry("inclineAngle");
    private final NetworkTableEntry inclineDirection = table.getEntry("inclineDirection");

    private final NetworkTableEntry pitch = table.getEntry("pitch");
    private final NetworkTableEntry roll = table.getEntry("roll");

    private Pose2d pose;

    public Odometry(
            AHRS gyro,
            SwerveDriveOdometry odometry, SwerveModulePosition[] modulePositions) {
        this.gyro = gyro;
        this.odometry = odometry;
        this.modulePositions = modulePositions;
        this.pose = new Pose2d();

        inclineAngle.setDefaultDouble(0);
        inclineDirection.setDefaultDouble(0);
        pitch.setDefaultDouble(0);
        roll.setDefaultDouble(0);
    }

    public void update(SwerveModulePosition[] newPositions) {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        // Update the pose
        pose = odometry.update(gyroAngle, newPositions);

        modulePositions = newPositions;

        inclineAngle.setDouble(getInclineAngle().getDegrees());
        inclineDirection.setDouble(getInclineDirection().getDegrees());

        pitch.setDouble(getPitch().getDegrees());
        roll.setDouble(getRoll().getDegrees());
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), modulePositions, pose);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Pose2d getPose() {
        // System.out.println(odometry.getPoseMeters());
        return odometry.getPoseMeters();
    }

    
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();//.unaryMinus();
    }

    public Rotation2d getPitch() {
        // switched because of roborio placement
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Rotation2d getRoll() {
        // switched because of roborio placement
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getInclineAngle() {
        double y = getPitch().getRadians();
        double x = getRoll().getRadians();

        double rad = (Math.PI / 2) - Math.acos(Math.sqrt(
            (Math.pow(Math.cos(y) * Math.sin(x), 2) + Math.pow(Math.sin(y) * Math.cos(x), 2)) /
            (Math.pow(Math.cos(y), 2) + Math.pow(Math.sin(y) * Math.cos(x), 2))));
        // System.out.println(rad * 180 / Math.PI);

        // double rad = Math.atan(
        //     y / (Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2))));
        
        return Rotation2d.fromRadians(rad).unaryMinus();
    }

    public Rotation2d getInclineDirection() {
        double y = getPitch().getRadians();
        double x = getRoll().getRadians();
        if (y == 0 && x == 0)
            return new Rotation2d();
        else {
            double rad = Math.atan(Math.tan(x) / Math.tan(y));
            if (y < 0)
                return Rotation2d.fromRadians(rad + Math.PI);//.unaryMinus();
            return Rotation2d.fromRadians(rad);//.unaryMinus();
        }
    }
}