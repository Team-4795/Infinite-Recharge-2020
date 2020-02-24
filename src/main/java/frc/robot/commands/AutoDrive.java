/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

import frc.robot.Robot;

public class AutoDrive extends CommandBase {

  private NetworkTable table;
  
  public AutoDrive() {
    addRequirements(Robot.drivebase);
    table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double var1 = table.getEntry("test").getDouble(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}