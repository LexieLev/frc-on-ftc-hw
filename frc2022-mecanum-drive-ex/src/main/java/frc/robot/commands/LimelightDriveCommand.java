package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightDriveCommand extends CommandBase {
    

    private DriveSubsystem mDrive;

    
    public LimelightDriveCommand(DriveSubsystem mDrive){
        this.mDrive = mDrive;
        addRequirements(mDrive);
    }
    @Override
    public void execute(){

        mDrive.DriveLimelight();
    }
    @Override
    public void end(boolean interrupted){
        mDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    }
}


