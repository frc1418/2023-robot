package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Odometry;

public class SwerveDriveSubsystem extends SubsystemBase {

  private final WheelSubsystem backRight;
  private final WheelSubsystem backLeft;
  private final WheelSubsystem frontRight;
  private final WheelSubsystem frontLeft;

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

  private final NetworkTableEntry ntIsFieldCentric = table.getEntry("isFieldCentric");
  private final NetworkTableEntry ntVelocity = table.getEntry("wheelvelocity");
  private final NetworkTable odometryTable = ntInstance.getTable("/common/Odometry");
  private final NetworkTableEntry ntOdometryPose = odometryTable.getEntry("odometryPose");
  private final SwerveDriveKinematics kinematics;
  private final Odometry odometry;
  private final PIDController rotationController = new PIDController(0.04, 0, 0);
  public boolean fieldCentric = true;
  private double lockedRot;

  public SwerveDriveSubsystem(
      WheelSubsystem backRight,
      WheelSubsystem backLeft,
      WheelSubsystem frontRight,
      WheelSubsystem frontLeft,
      SwerveDriveKinematics kinematics,
      Odometry odometry) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;

    this.kinematics = kinematics;
    this.odometry = odometry;

    this.ntIsFieldCentric.setBoolean(fieldCentric);

    this.lockedRot = odometry.getHeading();
  }

  public void drive(double x, double y, double rot) {

    if (rot == 0) {
      rot = rotationController.calculate(odometry.getHeading(), lockedRot);
    } else {
      lockedRot = odometry.getHeading();
    }

    ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);

    if (fieldCentric) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometry.getRotation2d());
    }

    drive(speeds);
  }

  public void drive(ChassisSpeeds speeds) {
    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    drive(moduleStates);
  }

  public void drive(SwerveModuleState[] moduleStates) {
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

  public void strafe(Rotation2d direction, double speed) {
    drive(speed * Math.cos(direction.getRadians()), speed * Math.sin(direction.getRadians()), 0);
  }

  public Command turtleCommand() {
    return run(this::turtle);
  }

  private void turtle() {
    SwerveModuleState negative45 = new SwerveModuleState(0, new Rotation2d(-Math.PI / 4));
    SwerveModuleState positive45 = new SwerveModuleState(0, new Rotation2d(Math.PI / 4));

    SwerveModuleState[] moduleStates =
        new SwerveModuleState[] {negative45, positive45, positive45, negative45};
    drive(moduleStates);
  }

  @Override
  public void periodic() {
    odometry.update(getPositions());

    ntOdometryPose.setString(odometry.getPose().toString());

    frontLeft.periodic();
    frontRight.periodic();
    backLeft.periodic();
    backRight.periodic();

    if (DriverStation.isAutonomousEnabled()) {
      lockedRot = odometry.getHeading();
    }
  }

  private void toggleFieldCentric() {
    fieldCentric = !fieldCentric;
    ntIsFieldCentric.setBoolean(fieldCentric);
  }

  public boolean getFieldCentric() {
    return fieldCentric;
  }

  public void setFieldCentric(boolean fieldCentric) {
    this.fieldCentric = fieldCentric;
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getSwerveModulePosition(),
      frontRight.getSwerveModulePosition(),
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()
    };
  }

  public CommandBase disableFieldCentricCommand() {
    return this.runOnce(() -> setFieldCentric(false));
  }

  public CommandBase enableFieldCentricCommand() {
    return this.runOnce(() -> setFieldCentric(true));
  }

  public CommandBase toggleFieldCentricCommand() {
    return this.runOnce(this::toggleFieldCentric);
  }

  public Command stop() {
    return runOnce(() -> drive(0, 0, 0));
  }

  public void resetLockRot() {
    lockedRot = odometry.getHeading();
  }
}
