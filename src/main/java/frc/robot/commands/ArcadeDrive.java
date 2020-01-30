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

public class ArcadeDrive extends CommandBase {

  /*private double maxVel;
  private double pastVel;
  private double maxAccel;*/
  private Boolean reversed;

  public ArcadeDrive() {
    addRequirements(Robot.drivebase);
    reversed = false;
  }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        double throttle = 0.85;// - 0.65 * Robot.oi.getMainRightTrigger();
        double turn = Robot.oi.getMainRightJoyX() * (Robot.oi.getMainLeftJoyY() == 0 ? 0.6 : 0.35);
        // if (Robot.oi.getMainRightTrigger() > 0.5) {
        //     turn *= 1.3;
        // }
        if (reversed) {
            throttle *= -1;
            turn *= -1;
        }

        SmartDashboard.putBoolean("Reversed Drivebase", reversed);
        Robot.drivebase.setMotors((Robot.oi.getMainLeftJoyY() - turn) * throttle,
                (Robot.oi.getMainLeftJoyY() + turn) * throttle);

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
    public void end(boolean interrupted) {
    }
}