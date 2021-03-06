/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.subsystems.Spinner.SpinnerColor;

public class SpinnerControl extends CommandBase {
  static SpinnerColor getGameData() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        // Note that the color is offset as our sensor is placed different to the FMS's sensor.
        case 'B': return SpinnerColor.RED;
        case 'G': return SpinnerColor.YELLOW;
        case 'R': return SpinnerColor.BLUE;
        case 'Y': return SpinnerColor.GREEN;
      }
    }
    return SpinnerColor.NONE;
  }

  public SpinnerControl() {
    addRequirements(Robot.spinner);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (Robot.rc.main.pressedY()) {
      SpinnerColor target = getGameData();
      if (target == SpinnerColor.NONE) {
        Robot.spinner.turnCycles(4);
      } else {
        Robot.spinner.turnTo(target);
      }
    } else if (Robot.rc.main.releasedY()) {
      Robot.spinner.disable();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
