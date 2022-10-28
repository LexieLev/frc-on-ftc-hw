package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DriveSubsystem mDrive;

    public DefaultDrive (DoubleSupplier xSupplier, DoubleSupplier ySupplier, DriveSubsystem mDrive) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.mDrive = mDrive;
        addRequirements(mDrive);
    }


    @Override
    public void initialize() {
      //m_drive.resetEncoders();
    }
  
    @Override
    public void execute() {

        double y = ySupplier.getAsDouble();
        double x = xSupplier.getAsDouble();
        mDrive.driveCartesian(y, x, 0.0, 0.0);
    }
  
    @Override
    public void end(boolean interrupted) {
      mDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    }
  
    // @Override
    // public boolean isFinished() {
    // //   return mTimer.get() >= mTimeS;
    // }


}
