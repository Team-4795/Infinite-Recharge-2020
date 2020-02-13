/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class ManualArmControl extends CommandBase {

  private double throttle;

  public ManualArmControl() {
    requires(Robot.Arm);
  }

  private void requires(Object arm) {
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    throttle = (0.55 - (0.3 * Robot.oi.getArmRightTrigger()));
    ((Arm) Robot.Arm).actuate(Robot.oi.getArmLeftJoyY() * throttle);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}