package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.SPI;

public class DefaultDrive extends CommandBase {
    
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier rSupplier;
    private DriveSubsystem mDrive;
    
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  private final BooleanSupplier isFieldRelativeSupplier;

    public DefaultDrive (DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, DriveSubsystem mDrive, BooleanSupplier isFieldRelativeSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.mDrive = mDrive;
        addRequirements(mDrive);
        this.isFieldRelativeSupplier = isFieldRelativeSupplier;
    }


    @Override
    public void initialize() {
      //m_drive.resetEncoders();

      
    }
  
    @Override
    public void execute() {

        double y = ySupplier.getAsDouble();
        double x = xSupplier.getAsDouble();
        double r = rSupplier.getAsDouble();
       
        mDrive.driveCartesian(y, x, r, 
        isFieldRelativeSupplier.getAsBoolean()
            ?  gyro.getAngle()
            :  0);
        
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
