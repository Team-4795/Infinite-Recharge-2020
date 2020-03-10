/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualArmControl extends CommandBase {

  private boolean twoManLock;
  public ManualArmControl() {
    addRequirements(Robot.arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // SmartDashboard.putNumber("AMR AMR AMR", Robot.arm.getPos());
    // SmartDashboard.putNumber("SERVO", Arm.servo.getAngle());
    // SmartDashboard.putNumber("LEFT JOY Y", -Robot.rc.arm.leftJoystick().y);
    Robot.arm.setRoller((Robot.rc.arm.leftBumper() ? 1 : 0) - (Robot.rc.arm.rightBumper() ? 1 : 0));
    if (Robot.arm.armPositionMode == 1 && Robot.arm.pidEnabled && Robot.rc.arm.rightTrigger() > 0.3) {
      // SmartDashboard.putNumber("POWER", Robot.rc.arm.rightTrigger());
      // Robot.arm.setVoltage(0.02*RobotController.getBatteryVoltage());
      Robot.arm.setArm(0.04, true);
    } else {
      double speed = -Robot.rc.arm.leftJoystick().y * 0.5;
      if (speed > 0.1 || speed < -0.1) Robot.arm.pidEnabled = false;
      if (Robot.rc.arm.dPad() == 0) {
        Robot.arm.pidEnabled = true;
        Robot.arm.setPosition(0);
      } else if (Robot.rc.arm.dPad() == 180 || Robot.rc.main.getA()) {
        Robot.arm.pidEnabled = true;
        Robot.arm.setPosition(1);
      }


      if (Robot.arm.armPositionMode == 1 && Robot.rc.arm.leftBumper()) {
        Robot.arm.setArm(0.04, true);
        return;
      }

      if (Robot.arm.pidEnabled) {
        Robot.arm.goToPosition();
      } else {
        Robot.arm.setArm(speed, false);
      }

      if (Robot.drivebase.climbTime) {
        Robot.arm.setPosition(1);
        Robot.arm.goToPosition();
      }
    }

    if (Robot.rc.main.getB()) {
      Robot.arm.setArm(-1, true);
    }
    SmartDashboard.putNumber("POV", Robot.rc.arm.dPad());

    
  }
  
  
  
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}