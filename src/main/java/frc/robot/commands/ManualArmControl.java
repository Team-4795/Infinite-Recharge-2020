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

  private double throttle;

  public ManualArmControl() {
    addRequirements(Robot.arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    throttle = 0.55 - 0.3 * Robot.oi.arm.rightTrigger();
    // Robot.arm.setArm(throttle);
    // Robot.arm.actuate(Robot.oi.arm.leftJoystick().y * throttle);
    // if (Robot.oi.main.getB()) {
    //   Robot.arm.spinner.set(ControlMode.PercentOutput, 0.9);
    // }
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}