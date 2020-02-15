/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivebase;

public class ArcadeDrive extends CommandBase {

  /*private double maxVel;
  private double pastVel;
  private double maxAccel;*/
  private final Boolean reversed;
  public Drivebase drivebase;

  public ArcadeDrive(final Drivebase drive) {
  
    addRequirements(drive);
    drivebase = drive;
    reversed = false;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle = 0.85;// - 0.65 * Robot.oi.getMainRightTrigger();
    double turn = Robot.rc.getMainRightJoyX() * (Robot.rc.getMainLeftJoyY() == 0 ? 0.6 : 0.35);
    // if (Robot.oi.getMainRightTrigger() > 0.5) {
    // turn *= 1.3;
    // }
    if (reversed) {
      throttle *= -1;
      turn *= -1;
    }

    SmartDashboard.putBoolean("Reversed Drivebase", reversed);
    drivebase.setMotors((Robot.rc.getMainLeftJoyY() - turn) * throttle, (Robot.rc.getMainLeftJoyY() + turn) * throttle);

    /*
     * maxVel = Math.abs(Robot.drivebase.getLeftVelocity()) > Math.abs(maxVel) ?
     * Robot.drivebase.getLeftVelocity() : maxVel;
     * SmartDashboard.putNumber("Max Velocity", maxVel); maxAccel =
     * Math.abs(Math.abs(Robot.drivebase.getLeftVelocity()) - Math.abs(pastVel)) >
     * Math.abs(maxAccel) ? (Math.abs(Robot.drivebase.getLeftVelocity()) -
     * Math.abs(pastVel)) / 0.05 : maxAccel; pastVel =
     * Robot.drivebase.getLeftVelocity();
     */
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(final boolean interrupted) {
    }
}