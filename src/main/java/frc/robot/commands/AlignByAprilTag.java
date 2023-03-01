package frc.robot.commands;

import com.kauailabs.navx.IMUProtocol.YPRUpdate;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.common.Odometry;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignByAprilTag extends CommandBase {

    PIDController speedXController;
    PIDController speedYController;

    PIDController speedRotController;

    SwerveDriveSubsystem swerveDrive;
    LimelightSubsystem limelight;
    Odometry odometry;

    double targetX;
    double targetY;

    boolean startedFieldCentric;

    ProfiledPIDController speedController;

    public AlignByAprilTag(SwerveDriveSubsystem swerveDrive, LimelightSubsystem limelight, Odometry odometry, double targetX, double targetY) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        speedXController = new PIDController(1.7, 0, 0);
        speedYController = new PIDController(1, 0, 0);
        speedRotController = new PIDController(0.05, 0, 0);
        speedController = new ProfiledPIDController(1, 0, 0, new Constraints(1.5,0.2));

        speedRotController.enableContinuousInput(-180, 180);

        

        addRequirements(swerveDrive);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.startedFieldCentric = swerveDrive.getFieldCentric();
        this.swerveDrive.setFieldCentric(false);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Pose2d robotPose = new Pose2d(
            new Translation2d(odometry.getPose().getY(), odometry.getPose().getX()),
            odometry.getPose().getRotation().unaryMinus());

        Pose2d targetPose = new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(limelight.getTargetRotation()));

        // System.out.println("X DIFFERENCE: " + (robotPose.getX()));
        // System.out.println("Y DIFFERENCE: " + (robotPose.getY()));
        // System.out.println("R DIFFERENCE: " + (robotPose.getRotation().getDegrees()));

        double x;
        double y;
        double rot;

        double dx = robotPose.getX() - targetPose.getX();
        double dy =  robotPose.getY() - targetPose.getY();

        double distance = Math.hypot(dx, dy);
        double angle = Math.atan2(dx, dy) * 180 / Math.PI; //targetPose.getRotation().getDegrees() - robotPose.getRotation().getDegrees();
        // if (limelight.getIsDetecting()){
        //     odometry.reset(new Pose2d(new Translation2d(limelight.getXDistance(), limelight.getYDistance()),
        //         limelight.getRotation()));
        // }
        // System.out.println(targetX);

        x = 0;//speedXController.calculate(odometry.getPose().getY(), targetX);
        y = 0;//speedYController.calculate(odometry.getPose().get^X(), targetY);
        rot = -speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), limelight.getTargetRotation());
        
        double speed = speedController.calculate(distance, 0);

        // System.out.println("DX: " + dx);
        // System.out.println("DY: " + dy);
        System.out.println("DISTANCE: " + distance);

        // if(limelight.getIsDetecting()){
            // System.out.println(angle);
            swerveDrive.strafe(Rotation2d.fromDegrees(180+angle), speed);
            // swerveDrive.drive(y, x, rot);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        this.swerveDrive.setFieldCentric(startedFieldCentric);
        swerveDrive.drive(0, 0, 0);
        
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
    
}
