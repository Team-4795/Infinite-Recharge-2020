/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// test comment
package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.ArmToPosition;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
public class RobotContainer {

  private static final double DEADZONE = 0.15;

  public Joystick main;
  private final Drivebase drive; 
  private final double maxVel;
  private final double maxAcc;
  private final double b;
  private final double zeta;
  // private JoystickButton XButton, YButton, AButton, BButton, RightBumper;
  // private double value;
  // private POVButton MainDPadDown, MainDPadUp;

  public RobotContainer() { 
    String trajectoryJSON = "PathWeaver/pathweaver.json" ;
    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }
    drive = new Drivebase();
    main = new Joystick(Constants.MAIN_CONTROLLER);
    b = 2.0;
    zeta = 0.7;
    maxVel = 3.0;
    maxAcc = 2.0;
    // ARM_CONTROLLER = new Joystick(Constants.ARM_CONTROLLER);

    // YButton = new JoystickButton(MAIN_CONTROLLER, 4);
    // AButton = new JoystickButton(MAIN_CONTROLLER, 1);
    // XButton = new JoystickButton(MAIN_CONTROLLER, 3);
    // BButton = new JoystickButton(MAIN_CONTROLLER, 2);
    // ArmBButton = new JoystickButton(ARM_CONTROLLER, 2);
    // ArmDPadUp = new POVButton(ARM_CONTROLLER, 0);
    // ArmDPadDown = new POVButton(ARM_CONTROLLER, 180);
    // RightBumper = new JoystickButton(MAIN_CONTROLLER, 6);
    // MainDPadUp = new POVButton(MAIN_CONTROLLER, 0);
    // MainDPadDown = new POVButton(MAIN_CONTROLLER, 180);
    // ArmDPadRight = new POVButton(ARM_CONTROLLER, 90);
    // ArmLeftBumper = new JoystickButton(ARM_CONTROLLER, 5);
    // ArmRightBumper = new JoystickButton(ARM_CONTROLLER, 6);

    // ArmDPadDown.whenPressed(new ArmToPosition(-78.5));
    // ArmDPadUp.whenPressed(new ArmToPosition(-17.38));
    // ArmDPadRight.whenPressed(new ArmToPosition(-42.3));

    //drivetrainOverride.whileActive(new ArcadeDrive());

    //AButton.whenPressed(new TurnToLine(5));
  }

  // Drivebase control

  public double getMainLeftJoyX() {
    double value = main.getRawAxis(0);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }
  
  public double getMainLeftJoyY() {
    double value = main.getRawAxis(1);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }
  
  //For tankdrive control (unused)
  public double getMainRightJoyY() {
    double value = main.getRawAxis(2);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }

  //Drivebase control
  public double getMainRightJoyX() {
    double value = main.getRawAxis(3);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }

  //Drivebase throttle
  // public double getMainRightTrigger() {
  //   double value = MAIN_CONTROLLER.getRawAxis(3);
  //   return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  // }

  //Climber wheel actuation & outtaking
  // public double getMainLeftTrigger() {
  //   double value = MAIN_CONTROLLER.getRawAxis(2);
  //   return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  // }

  public boolean getMainBButtonPressed() {
    return main.getRawButtonPressed(2);
  }

  public boolean getMainBButton() {
    return main.getRawButton(2);
  }

  public boolean getMainLeftBumperPressed() {
    return main.getRawButtonPressed(5);
  }

  // public boolean getArmLeftBumper() {
  //   return ARM_CONTROLLER.getRawButton(5);
  // }

  // public boolean getArmRightBumper() {
  //   return ARM_CONTROLLER.getRawButton(6);
  // }

  // //toggles which way is "forward" for drivebase
  // public boolean getMainRightBumperPressed() {
  //   return MAIN_CONTROLLER.getRawButtonPressed(6);
  // }

  // public boolean getArmXButton() {
  //   return ARM_CONTROLLER.getRawButton(3);
  // }

  // public boolean getArmAButton() {
  //   return ARM_CONTROLLER.getRawButton(1);
  // }

  // public boolean getArmYButton() {
  //   return ARM_CONTROLLER.getRawButton(4);
  // }

  public boolean getMainYButtonPressed() {
    return main.getRawButtonPressed(4);
  }

  //Arm control
  // public double getArmLeftJoyY() {
  //   double value = ARM_CONTROLLER.getRawAxis(1);
  // return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  // }

  //Arm throttle
  // public double getArmRightTrigger() {
  //   double value = ARM_CONTROLLER.getRawAxis(3);
  // return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  // }

  // public double getArmLeftTrigger() {
  //   double value = ARM_CONTROLLER.getRawAxis(2);
  // return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  // }

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAcc);
    config.setKinematics(drive.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),config);
    RamseteCommand command = new RamseteCommand(trajectory, drive::getPose, new RamseteController(b, zeta), drive.getFeedForward(), drive.getKinematics(), drive::getSpeeds, drive.getLeftPID(), drive.getRightPID(), drive::setOutput, drive);
    return command;
  }
}