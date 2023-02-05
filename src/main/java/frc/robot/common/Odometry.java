package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



public class Odometry {
    private final SwerveDriveOdometry odometry;
    private final Gyro gyro;
    private SwerveModulePosition[] modulePositions;

    private Pose2d pose;

    public Odometry(
            Gyro gyro,
            SwerveDriveOdometry odometry, SwerveModulePosition[] modulePositions) {
        this.gyro = gyro;
        this.odometry = odometry;
        this.modulePositions = modulePositions;
        this.pose = new Pose2d();
    }

    public void update(SwerveModulePosition[] newPositions) {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        // Update the pose
        pose = odometry.update(gyroAngle, newPositions);

        modulePositions = newPositions;
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
}