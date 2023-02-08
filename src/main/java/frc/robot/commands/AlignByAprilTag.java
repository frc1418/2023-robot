package frc.robot.commands;

import com.kauailabs.navx.IMUProtocol.YPRUpdate;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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

    public AlignByAprilTag(SwerveDriveSubsystem swerveDrive, LimelightSubsystem limelight, Odometry odometry, double targetX, double targetY) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        speedXController = new PIDController(1.1, 0, 0);
        speedYController = new PIDController(1.1, 0, 0);
        speedRotController = new PIDController(0.001, 0, 0);

        addRequirements(swerveDrive);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double x;
        double y;
        double rot;

        // if (limelight.getIsDetecting()){
        //     odometry.reset(new Pose2d(new Translation2d(limelight.getXDistance(), limelight.getYDistance()),
        //         limelight.getRotation()));
        // }
        x = speedXController.calculate(odometry.getPose().getX(), targetX);
        y = speedYController.calculate(odometry.getPose().getY(), targetY);
        // rot = speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), 0);
       

        // if(limelight.getIsDetecting()){
            swerveDrive.drive(x, y, 0);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        swerveDrive.drive(0, 0, 0);
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
    
}
