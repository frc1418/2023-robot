package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.common.Odometry;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignWithSubstationCommand extends CommandBase {

    PIDController speedXController;
    PIDController speedYController;

    PIDController speedRotController;

    SwerveDriveSubsystem swerveDrive;
    LimelightSubsystem limelight;

    public AlignWithSubstationCommand(SwerveDriveSubsystem swerveDrive, LimelightSubsystem limelight) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
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
        double x = -speedXController.calculate(limelight.getXDistance(), 0);
        double y = speedYController.calculate(limelight.getYDistance(), -0.9);
        double rot = -speedRotController.calculate(limelight.getRotation(), 0);

        System.out.println(rot);
        swerveDrive.drive(y, x, rot);

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
