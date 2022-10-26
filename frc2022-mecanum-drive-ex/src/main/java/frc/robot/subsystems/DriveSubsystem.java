package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.MecanumDriveCTRE;
import frc.robot.Constants.Drive.MotorPorts;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDriveCTRE mDrive;
    private TalonSRX mFrontLeftTalon;
    private TalonSRX mRearLeftTalon;
    private TalonSRX mFrontRightTalon;
    private TalonSRX mRearRightTalon;
    private TalonSRXConfiguration mDriveTalonSRXConfigAll;
    private MecanumDriveCTRE mRobotDrive;
    private final PIDController pidController;
    private final CANCoder armEncoder;

    public DriveSubsystem() {
        mFrontLeftTalon = new TalonSRX(MotorPorts.kFrontLeftId);
        mRearLeftTalon = new TalonSRX(MotorPorts.kRearLeftId);
        mFrontRightTalon = new TalonSRX(MotorPorts.kFrontRightId);
        mRearRightTalon = new TalonSRX(MotorPorts.kRearRightId);
        List<TalonSRX> mDriveTalons = new ArrayList<>(Arrays.asList(
        mFrontLeftTalon, 
        mRearLeftTalon, 
        mFrontRightTalon, 
        mRearRightTalon
        ));
    // default motor settings
    mDriveTalonSRXConfigAll = new TalonSRXConfiguration();
    mDriveTalonSRXConfigAll.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    mDriveTalonSRXConfigAll.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
    //mDriveTalons.stream().map(t -> t.configAllSettings(mDriveTalonSRXConfigAll));
    //if we don't care about error codes here we can do this too: 
    mDriveTalons.forEach(talon -> talon.configAllSettings(mDriveTalonSRXConfigAll));
    // invert the right side motors
    mFrontRightTalon.setInverted(true);
    mRearRightTalon.setInverted(true);
    // coast the drive motors - not part of configAllSettings
    mDriveTalons.forEach(talon -> talon.setNeutralMode(NeutralMode.Coast));
    // configure velocity control
    mDriveTalons.forEach(
      talon -> talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
    );
    mDriveTalons.forEach(talon -> talon.config_kF(0, 3.83625));
    mDriveTalons.forEach(talon -> talon.config_kP(0, 2.1));
    mDriveTalons.forEach(talon -> talon.config_kI(0, 0.002));

    mRobotDrive = new MecanumDriveCTRE(mFrontLeftTalon, mRearLeftTalon, mFrontRightTalon, mRearRightTalon);
    // adjust for 117rpm in front and 312rpm in back
    mRobotDrive.setMotorCoeff(1, 0.375, 1, 0.375);
    // enable velocity control - max scale in ticks/100ms
    mRobotDrive.setControlMode(ControlMode.Velocity, 260);
    }

   public void driveMecanum(DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRot)
   {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
        mRobotDrive.driveCartesian(ySpeed, xSpeed, zRot, 0.0);
    }
}
