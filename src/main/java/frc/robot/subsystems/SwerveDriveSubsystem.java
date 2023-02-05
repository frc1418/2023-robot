package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Odometry;

public class SwerveDriveSubsystem extends SubsystemBase{

    private WheelSubsystem backRight;
    private WheelSubsystem backLeft;
    private WheelSubsystem frontRight;
    private WheelSubsystem frontLeft;
    
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry ntBackRightAngleEncoder = table.getEntry("backRightAngleEncoder");
    private final NetworkTableEntry ntBackLeftAngleEncoder = table.getEntry("backLeftAngleEncoder");
    private final NetworkTableEntry ntFrontRightAngleEncoder = table.getEntry("frontRightAngleEncoder");
    private final NetworkTableEntry ntFrontLeftAngleEncoder = table.getEntry("frontLeftAngleEncoder");

    private final NetworkTableEntry ntIsFieldCentric = table.getEntry("isFieldCentric");

    private final NetworkTable odometryTable = ntInstance.getTable("/common/Odometry");
    private final NetworkTableEntry ntOdometryPose = odometryTable.getEntry("odometryPose");
    private final NetworkTableEntry ntVelocity = table.getEntry("wheelvelocity");



    private SwerveDriveKinematics kinematics;
    private Odometry odometry;
    
    public boolean fieldCentric = false;

    public SwerveDriveSubsystem (WheelSubsystem backRight, WheelSubsystem backLeft, WheelSubsystem frontRight, WheelSubsystem frontLeft, SwerveDriveKinematics kinematics, Odometry odometry) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;

        this.kinematics = kinematics;
        this.odometry = odometry;

        this.ntIsFieldCentric.setBoolean(fieldCentric);
    }


    public void drive (double x, double y, double rot) {
        

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);
        // ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

        if (fieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometry.getRotation2d());
        }
        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        drive(moduleStates);
    }

    public void drive (SwerveModuleState[] moduleStates) {
        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        ntVelocity.setDouble(frontLeftState.speedMetersPerSecond);

        frontLeft.drive(frontLeftState);
        frontRight.drive(frontRightState);
        backLeft.drive(backLeftState);
        backRight.drive(backRightState);

    }

    @Override
    public void periodic() {
        odometry.update(getPositions());

        ntOdometryPose.setString(odometry.getPose().toString());

        ntBackLeftAngleEncoder.setDouble(backLeft.getEncoderPosition());
        ntBackRightAngleEncoder.setDouble(backRight.getEncoderPosition());
        ntFrontLeftAngleEncoder.setDouble(frontLeft.getEncoderPosition());
        ntFrontRightAngleEncoder.setDouble(frontRight.getEncoderPosition());
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
        ntIsFieldCentric.setBoolean(fieldCentric);
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
          };
    }

}
