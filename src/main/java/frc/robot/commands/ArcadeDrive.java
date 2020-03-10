/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import java.util.Date;

public class ArcadeDrive extends CommandBase {

  private boolean reversed;
  private boolean ballz;
  private Date currentTime;

  public ArcadeDrive() {
    addRequirements(Robot.drivebase);
    reversed = false;
    ballz = true;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    currentTime = new Date();
    if (Robot.rc.main.pressedY()) {
      // SmartDashboard.putNumber("DATE_TIME", Robot.startTime.getTime() - currentTime.getTime()); 
      Robot.drivebase.climbTime = !Robot.drivebase.climbTime;
    }
    double throttle = Robot.rc.main.rightTrigger();
    double forward = Robot.rc.main.leftJoystick().y * (0.85 - 0.65 * throttle);
    double turn = Robot.rc.main.rightJoystick().x
      * (1 + 0.3 * throttle) * (0.85 - 0.65 * throttle) // maybe replace this with 0.85 - 0.55 * throttle instead of a quadratic 
      * (forward == 0 ? 0.65 : 0.5);

    if (Robot.rc.main.rightBumperPressed()) reversed = !reversed;
    if (reversed) forward *= -1;

    if (Robot.rc.arm.pressedY()) ballz = true;
    if (Robot.rc.arm.pressedX()) ballz = false;
    SmartDashboard.putBoolean("target_balls", ballz);

    if (ballz) turn += SmartDashboard.getNumber("ball_x", 0) * Robot.rc.main.leftTrigger() / 4.5 * (1 - (forward / 2));
    if (!ballz) turn += SmartDashboard.getNumber("target_x", 0) * Robot.rc.main.leftTrigger() / 4.5 * (1 - (forward / 2));
    
    // if (Robot.drivebase.climbTime) turn *= 0.5;
    Robot.drivebase.setMotors(forward * 0.9 - turn, forward + turn);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}