package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriverSubsystem extends SubsystemBase{

    private WheelSubsystem backRight;
    private WheelSubsystem backLeft;
    private WheelSubsystem frontRight;
    private WheelSubsystem frontLeft;

    private SwerveDriveKinematics kinematics;

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

        System.out.println("BR: " + backRight.getAngleMotor().getEncoder().getPosition());
        //backRight.getEncoder().getDistance();


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
