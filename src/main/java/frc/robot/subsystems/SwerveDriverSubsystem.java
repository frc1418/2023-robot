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

    private final NetworkTableEntry backRightEncoderOutput = table.getEntry("backRightEncoderOutput");
    private final NetworkTableEntry backLeftEncoderOutput = table.getEntry("backLeftEncoderOutput");
    private final NetworkTableEntry frontRightEncoderOutput = table.getEntry("frontRightEncoderOutput");
    private final NetworkTableEntry frontLeftEncoderOutput = table.getEntry("frontLeftEncoderOutput");



    public SwerveDriverSubsystem (WheelSubsystem backRight, WheelSubsystem backLeft, WheelSubsystem frontRight, WheelSubsystem frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        

        kinematics = new SwerveDriveKinematics(
            frontLeft.location, frontRight.location, backLeft.location, backRight.location);
    }


    public void drive (double x, double y, double rot) {
        

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);
        // ChassisSpeeds speeds = new ChassisSpeeds(0, 0.3, 0);

        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState backLeftState = moduleStates[0];
        SwerveModuleState backRightState = moduleStates[1];
        SwerveModuleState frontLeftState = moduleStates[2];
        SwerveModuleState frontRightState = moduleStates[3];

        frontLeft.drive(frontLeftState);
        frontRight.drive(frontRightState);
        backLeft.drive(backLeftState);
        backRight.drive(backRightState);


        backLeftAngleEncoder.setDouble(backLeft.getEncoderPosition());
        backRightAngleEncoder.setDouble(backRight.getEncoderPosition());
        frontLeftAngleEncoder.setDouble(frontLeft.getEncoderPosition());
        frontRightAngleEncoder.setDouble(frontRight.getEncoderPosition());

        backLeftEncoderOutput.setDouble(backLeft.getEncoderOutput());
        backRightEncoderOutput.setDouble(backRight.getEncoderOutput());
        frontLeftEncoderOutput.setDouble(frontLeft.getEncoderOutput());
        frontRightEncoderOutput.setDouble(frontRight.getEncoderOutput());


    }

    public void resetEncoders() {
        frontLeft.getEncoder().reset();
        frontLeft.getAngleMotor().getEncoder().setPosition(0);

        frontRight.getEncoder().reset();
        frontRight.getAngleMotor().getEncoder().setPosition(0);

        backLeft.getEncoder().reset();
        backLeft.getAngleMotor().getEncoder().setPosition(0);

        backLeftAngleEncoder.setDouble(backLeft.getEncoderPosition());
        backRightAngleEncoder.setDouble(backRight.getEncoderPosition());
        frontLeftAngleEncoder.setDouble(frontLeft.getEncoderPosition());
        frontRightAngleEncoder.setDouble(frontRight.getEncoderPosition());

        backLeftEncoderOutput.setDouble(backLeft.getangleSetpoint());
        backRightEncoderOutput.setDouble(backRight.getangleSetpoint());
        frontLeftEncoderOutput.setDouble(frontLeft.getangleSetpoint());
        frontRightEncoderOutput.setDouble(frontRight.getangleSetpoint());


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
