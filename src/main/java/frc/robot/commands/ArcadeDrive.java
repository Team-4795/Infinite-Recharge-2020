/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

public class ArcadeDrive extends CommandBase {

  private boolean reversed;
  // private double maxVel;
  // private double pastVel;
  // private double maxAccel;

  public ArcadeDrive() {
    addRequirements(Robot.drivebase);
    reversed = false;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle = 0.85 - 0.65 * Robot.oi.getMainRightTrigger();
    double forward = Robot.oi.getMainLeftJoyY() * throttle;
    double turn = Robot.oi.getMainRightJoyX()
      * (forward == 0 ? 0.6 : 0.35)
      * (1 + 0.3 * Robot.oi.getMainRightTrigger())
      * throttle;

    if (Robot.oi.getMainRightBumperPressed()) reversed = !reversed;
    if (reversed) {
      forward *= -1;
    }
    SmartDashboard.putBoolean("Reversed Drivebase", reversed);

    if (Robot.oi.getMainBButton()) {
      Robot.drivebase.drive(0.3, 0.2);
    } else {
      Robot.drivebase.setMotors(forward - turn, forward + turn);
    }

    // double vel = Robot.drivebase.getLeftVelocity();
    // maxVel = Math.abs(vel) > Math.abs(maxVel) ? vel : maxVel;
    // SmartDashboard.putNumber("Max Velocity", maxVel);
    // maxAccel = Math.abs(vel - pastVel) > Math.abs(maxAccel) ? (vel - pastVel) : maxAccel;
    // pastVel = vel;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}