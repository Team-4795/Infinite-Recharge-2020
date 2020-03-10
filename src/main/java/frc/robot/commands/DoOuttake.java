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

public class DoOuttake extends CommandBase {

  public DoOuttake() {
    addRequirements(Robot.arm);
  }

  @Override
  public void initialize() {
    // Robot.drivebase.resetEncoders();
    Robot.arm.setPosition(0);
  }

  @Override
  public void execute() {
    // Robot.drivebase.setMotors(0.0, 0.0);
    Robot.arm.goToPosition();
    Robot.arm.setRoller(-0.5);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
} 