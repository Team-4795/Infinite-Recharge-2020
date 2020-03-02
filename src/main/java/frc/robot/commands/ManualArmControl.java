/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class ManualArmControl extends CommandBase {

  public ManualArmControl() {
    addRequirements(Robot.arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.arm.setArm(Robot.rc.arm.leftJoystick().y);
    Robot.arm.setRoller((Robot.rc.arm.getY() ? 1 : 0) - (Robot.rc.arm.getX() ? 1 : 0));
//     throttle = 0.55 - 0.3 * Robot.rc.arm.rightTrigger();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
