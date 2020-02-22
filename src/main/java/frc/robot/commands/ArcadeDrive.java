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
// import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

public class ArcadeDrive extends CommandBase {

  private boolean reversed;
  // private int temp;
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
    double throttle = Robot.rc.main.rightTrigger();
    double forward = Robot.rc.main.leftJoystick().y * (0.85 - 0.65 * throttle);
    double turn = Robot.rc.main.rightJoystick().x
      * (1 + 0.3 * throttle) * (0.85 - 0.65 * throttle) // maybe replace this with 0.85 - 0.55 * throttle instead of a quadratic 
      * (forward == 0 ? 0.6 : 0.35);

    if (Robot.rc.main.rightBumperPressed()) reversed = !reversed;
    if (reversed) {
      forward *= -1;
    }
    SmartDashboard.putBoolean("Reversed Drivebase", reversed);
    SmartDashboard.putNumber("Left Encoder Count", Robot.drivebase.getLeftEncoderCount());
    SmartDashboard.putNumber("Right Encoder Count", Robot.drivebase.getRightEncoderCount());

    if (Robot.rc.main.getB()) {
      Robot.drivebase.drive(0.3, 0.2);
    } else {
      Robot.drivebase.setMotors(forward - turn, forward + turn);
    }

    if (Robot.rc.main.backButtonPressed()) {
      System.exit(0);
    }
    //   if (Robot.oi.getMainAButtonPressed()) {
    //     temp = Robot.drivebase.getLeftEncoderCount();
    //   }
    //   int relativeOver = 18148 - Robot.drivebase.getLeftEncoderCount() + temp;
    //   double speed = Math.min(0.2, relativeOver / 18148.0);
    //   Robot.drivebase.setMotors(speed, speed);

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