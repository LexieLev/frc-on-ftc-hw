// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

import frc.robot.MecanumDriveCTRE;



public class LimelightDrive extends CommandBase {
  private final MecanumDriveCTRE mDrive;


  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ta;
  private NetworkTableEntry ty; 

  private double previousPosX;
  private double previousPosY;
  private double currentX;
  private double currentY;

  private PIDController xController;
  private PIDController yController;

  private ShuffleboardTab shuffleboard;
  /**
   * Creates a new DriveTimed.
   *
   * @param timeS The number of inches the robot will drive
   * @param speedY The speed at which the robot will drive in Y (front+/back-)
   * @param speedX The speed at which the robot will drive in X (left+/right-)
   * @param speedRot The speed at which the robot will rotate
   * @param drive The drive subsystem on which this command will run
   */
  public LimelightDrive(
      MecanumDriveCTRE drive, NetworkTable table) {

    mDrive = drive;
    this.table = table; 

     shuffleboard = Shuffleboard.getTab("Limelight");
    
  }

  @Override
  public void initialize() {
    //m_drive.resetEncoders();
    mDrive.stopMotor();
    

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    previousPosX = tx.getDouble(0.0); 
    currentX = tx.getDouble(0.0);

    previousPosY = ty.getDouble(0.0);
    currentY = ty.getDouble(0.0);

    shuffleboard.addNumber("X", () -> tx.getDouble(0.0));
    shuffleboard.addNumber("Y", () -> ty.getDouble(0.0));
    shuffleboard.addNumber("A", () -> ta.getDouble(0.0));
  }

  @Override
  public void execute() {

    double differenceX = previousPosX - currentX; 
    double ldifferenceX = xController.calculate(differenceX);
    ldifferenceX = MathUtil.clamp(ldifferenceX, -1, 1);

    double differenceY = previousPosY - currentY;
    double ldifferenceY = yController.calculate(differenceY);
    ldifferenceY = MathUtil.clamp(ldifferenceY, -1, 1);

    mDrive.driveCartesian(ldifferenceY, ldifferenceX, 0);
  }


  private void periodic()
  {

    
  }
  @Override
  public void end(boolean interrupted) {
    mDrive.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}