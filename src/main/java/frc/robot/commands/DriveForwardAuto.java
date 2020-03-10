/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveForwardAuto extends CommandBase {
 private double distance;
  public DriveForwardAuto(double distance) {
    addRequirements(Robot.drivebase);
    this.distance = distance;
  }

  @Override
  public void initialize() {
    Robot.drivebase.resetEncoders();
  }

  @Override
  public void execute() {
    Robot.drivebase.driveFeet(distance);
    SmartDashboard.putBoolean("HASMOVED", Robot.drivebase.hasMoved);
  }
  
  @Override
  public boolean isFinished() {
    return Robot.drivebase.hasMoved;
  }

  @Override
  public void end(boolean interrupted) {
  }
} 