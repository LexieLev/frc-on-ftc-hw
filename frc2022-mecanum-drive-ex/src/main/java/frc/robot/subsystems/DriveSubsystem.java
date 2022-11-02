package frc.robot.subsystems;
import frc.robot.subsystems.WheelSpeeds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.MecanumDriveCTRE;
import frc.robot.Constants.Drive.MotorPorts;
import frc.robot.Constants.Drive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveSubsystem extends SubsystemBase {

    private TalonSRX mFrontLeftTalon;
    private TalonSRX mRearLeftTalon;
    private TalonSRX mFrontRightTalon;
    private TalonSRX mRearRightTalon;
    

    private double m_frontLeftCoeff = 1;
    private double m_rearLeftCoeff = 1;
    private double m_frontRightCoeff = 1;
    private double m_rearRightCoeff = 1;


    private ControlMode m_driveControlMode = ControlMode.PercentOutput;
    private double m_driveVelocityScale = 1;
    private double m_maxOutput = 1.0;

    public DriveSubsystem( TalonSRX mFrontLeftTalon, TalonSRX mRearLeftTalon, TalonSRX mFrontRightTalon, TalonSRX mRearRightTalon) {
        this.mFrontLeftTalon = mFrontLeftTalon;
        this.mRearLeftTalon = mRearLeftTalon;
        this.mFrontRightTalon = mFrontRightTalon;
        this.mRearRightTalon = mRearRightTalon;
      }

   public void driveMecanum(DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRot)
   {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
        // mRobotDrive.driveCartesian(ySpeed, xSpeed, zRot, 0.0);
    }

    public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
       
        
        ySpeed = MathUtil.applyDeadband(ySpeed, Drive.m_deadband);
        xSpeed = MathUtil.applyDeadband(xSpeed, Drive.m_deadband);
    
        var speeds = WheelSpeeds.driveCartesianIK(ySpeed, xSpeed, zRotation, gyroAngle);
    
        mFrontLeftTalon.set(m_driveControlMode, m_driveVelocityScale * speeds.frontLeft * m_maxOutput * m_frontLeftCoeff);
        mFrontRightTalon.set(m_driveControlMode, m_driveVelocityScale * speeds.frontRight * m_maxOutput * m_frontRightCoeff);
        mRearLeftTalon.set(m_driveControlMode, m_driveVelocityScale * speeds.rearLeft * m_maxOutput * m_rearLeftCoeff);
        mRearRightTalon.set(m_driveControlMode, m_driveVelocityScale * speeds.rearRight * m_maxOutput * m_rearRightCoeff);
    
        //feed();
      }
      public void setMotorCoeff(
        double frontLeftCoeff,
     
        double rearLeftCoeff,
        double frontRightCoeff,
        double rearRightCoeff) {
      m_frontLeftCoeff = frontLeftCoeff;
      m_rearLeftCoeff = rearLeftCoeff;
      m_frontRightCoeff = frontRightCoeff;
      m_rearRightCoeff = rearRightCoeff;
    }

      /**
   * Set control mode and velocity scale (opt)
   * 
   * @param controlMode control mode to use setting talon output
   * @param velocityScale velocity for full scale in ticks/100ms
   */
  public void setControlMode(ControlMode controlMode, double velocityScale) {
    m_driveControlMode = controlMode;
    m_driveVelocityScale = velocityScale;
  }
}

