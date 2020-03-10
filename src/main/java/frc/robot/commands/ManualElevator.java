/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.Date;

public class ManualElevator extends CommandBase {
  /**
   * Creates a new ManualElevator.
   */

   private Date currentTime;
  public ManualElevator() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.elevator); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(Robot.drivebase.climbTime) Robot.elevator.set((Robot.rc.arm.getB() ? -.7 : 0) + (Robot.rc.arm.getA() ? 1 : 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
