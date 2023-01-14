package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriverSubsystem extends SubsystemBase{

    private WheelSubsystem backRight;
    private WheelSubsystem backLeft;
    private WheelSubsystem frontRight;
    private WheelSubsystem frontLeft;

    private SwerveDriveKinematics kinematics;
    
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry backRightAngleEncoder = table.getEntry("backRightAngleEncoder");
    private final NetworkTableEntry backLeftAngleEncoder = table.getEntry("backLeftAngleEncoder");
    private final NetworkTableEntry frontRightAngleEncoder = table.getEntry("frontRightAngleEncoder");
    private final NetworkTableEntry frontLeftAngleEncoder = table.getEntry("frontLeftAngleEncoder");


    public SwerveDriverSubsystem (WheelSubsystem backRight, WheelSubsystem backLeft, WheelSubsystem frontRight, WheelSubsystem frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        

        kinematics = new SwerveDriveKinematics(
            frontLeft.location, frontRight.location, backLeft.location, backRight.location);
    }


    public void drive (double x, double y, double rot) {
        

        // ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0.3, 0);

        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        // frontLeft.drive(frontLeftState);
        // frontRight.drive(frontRightState);
        // backLeft.drive(backLeftState);
        // backRight.drive(backRightState);

        // backLeft.getAngleMotor().setVoltage(0.5);
        // backRight.getAngleMotor().setVoltage(0.5);
        // frontLeft.getAngleMotor().setVoltage(0.5);
        // frontRight.getAngleMotor().setVoltage(0.5);


        backLeftAngleEncoder.setDouble(backLeft.getEncoderPosition());
        backRightAngleEncoder.setDouble(backRight.getEncoderPosition());
        frontLeftAngleEncoder.setDouble(frontLeft.getEncoderPosition());
        frontRightAngleEncoder.setDouble(frontRight.getEncoderPosition());


    }

    public void resetEncoders() {
        frontLeft.getEncoder().reset();
        frontLeft.getAngleMotor().getEncoder().setPosition(0);

        frontRight.getEncoder().reset();
        frontRight.getAngleMotor().getEncoder().setPosition(0);

        backLeft.getEncoder().reset();
        backLeft.getAngleMotor().getEncoder().setPosition(0);

        backRight.getEncoder().reset();
        backRight.getAngleMotor().getEncoder().setPosition(0);
    }

}
