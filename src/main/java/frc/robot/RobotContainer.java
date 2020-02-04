/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.ArmToPosition;
import frc.robot.Constants;

public class RobotContainer {

  private static final double DEADZONE = 0.125;
  private static double eliminateDeadzone(double value) {
    return Math.abs(value) > DEADZONE
      ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE))
      : 0.0;
  }

  public Joystick main;
  // private JoystickButton XButton, YButton, AButton, BButton, RightBumper;
  // private double value;
  // private POVButton MainDPadDown, MainDPadUp;
  
  public RobotContainer() { 
    main = new Joystick(Constants.MAIN_CONTROLLER);
    // arm = new Joystick(Constants.ARM_CONTROLLER);

    // YButton = new JoystickButton(main, 4);
    // AButton = new JoystickButton(main, 1);
    // XButton = new JoystickButton(main, 3);
    // BButton = new JoystickButton(main, 2);
    // ArmBButton = new JoystickButton(arm, 2);
    // ArmDPadUp = new POVButton(arm, 0);
    // ArmDPadDown = new POVButton(arm, 180);
    // RightBumper = new JoystickButton(main, 6);
    // MainDPadUp = new POVButton(main, 0);
    // MainDPadDown = new POVButton(main, 180);
    // ArmDPadRight = new POVButton(arm, 90);
    // ArmLeftBumper = new JoystickButton(arm, 5);
    // ArmRightBumper = new JoystickButton(arm, 6);

    // ArmDPadDown.whenPressed(new ArmToPosition(-78.5));
    // ArmDPadUp.whenPressed(new ArmToPosition(-17.38));
    // ArmDPadRight.whenPressed(new ArmToPosition(-42.3));

    //drivetrainOverride.whileActive(new ArcadeDrive());

    //AButton.whenPressed(new TurnToLine(5));
  }

  // Controller joysticks
  public double getMainLeftJoyX() {
    return eliminateDeadzone(main.getRawAxis(0));
  }
  
  // Drivebase control (magnitude)
  public double getMainLeftJoyY() {
    return eliminateDeadzone(main.getRawAxis(1));
  }
  
  // Drivebase control (direction)
  public double getMainRightJoyX() {
    return eliminateDeadzone(main.getRawAxis(3));
  }

  // (unused)
  public double getMainRightJoyY() {
    return eliminateDeadzone(main.getRawAxis(2));
  }

  // Drivebase throttle, which slows down the robot but makes it turn faster
  public double getMainRightTrigger() {
    return eliminateDeadzone(main.getRawAxis(3));
  }

  // // Climber wheel actuation & outtaking
  // public double getMainLeftTrigger() {
  //   return eliminateDeadzone(main.getRawAxis(2));
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
  //   return arm.getRawButton(5);
  // }

  // public boolean getArmRightBumper() {
  //   return arm.getRawButton(6);
  // }

  // Toggles which way is "forward" for drivebase
  public boolean getMainRightBumperPressed() {
    return main.getRawButtonPressed(6);
  }

  // public boolean getArmXButton() {
  //   return arm.getRawButton(3);
  // }

  // public boolean getArmAButton() {
  //   return arm.getRawButton(1);
  // }

  // public boolean getArmYButton() {
  //   return arm.getRawButton(4);
  // }

  public boolean getMainYButtonPressed() {
    return main.getRawButtonPressed(4);
  }

  // // Arm control
  // public double getArmLeftJoyY() {
  //   return eliminateDeadzone(arm.getRawAxis(1));
  // }

  // // Arm throttle
  // public double getArmRightTrigger() {
  //   return eliminateDeadzone(arm.getRawAxis(3));
  // }

  // public double getArmLeftTrigger() {
  //   return eliminateDeadzone(arm.getRawAxis(2));
  // }

  public Command getAutonomousCommand() {
	  return null;
  }
}