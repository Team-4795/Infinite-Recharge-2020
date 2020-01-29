/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// test comment
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

  private static final double DEADZONE = 0.15;

  public Joystick MAIN_CONTROLLER;
  private JoystickButton XButton, YButton, AButton, BButton, RightBumper;
  private double value;
  private POVButton MainDPadDown, MainDPadUp;

  public RobotContainer() { 
    
  }

  public void init() {
    MAIN_CONTROLLER = new Joystick(Constants.MAIN_CONTROLLER);
    // ARM_CONTROLLER = new Joystick(Constants.ARM_CONTROLLER);

    YButton = new JoystickButton(MAIN_CONTROLLER, 4);
    AButton = new JoystickButton(MAIN_CONTROLLER, 1);
    XButton = new JoystickButton(MAIN_CONTROLLER, 3);
    BButton = new JoystickButton(MAIN_CONTROLLER, 2);
    // ArmBButton = new JoystickButton(ARM_CONTROLLER, 2);
    // ArmDPadUp = new POVButton(ARM_CONTROLLER, 0);
    // ArmDPadDown = new POVButton(ARM_CONTROLLER, 180);
    RightBumper = new JoystickButton(MAIN_CONTROLLER, 6);
    MainDPadUp = new POVButton(MAIN_CONTROLLER, 0);
    MainDPadDown = new POVButton(MAIN_CONTROLLER, 180);
    // ArmDPadRight = new POVButton(ARM_CONTROLLER, 90);
    // ArmLeftBumper = new JoystickButton(ARM_CONTROLLER, 5);
    // ArmRightBumper = new JoystickButton(ARM_CONTROLLER, 6);

    // ArmDPadDown.whenPressed(new ArmToPosition(-78.5));
    // ArmDPadUp.whenPressed(new ArmToPosition(-17.38));
    // ArmDPadRight.whenPressed(new ArmToPosition(-42.3));

    //drivetrainOverride.whileActive(new ArcadeDrive());

    //AButton.whenPressed(new TurnToLine(5));
  }
  //Drivebase control
  public double getMainLeftJoyX() {
    double value = MAIN_CONTROLLER.getRawAxis(0);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }
  
  public double getMainLeftJoyY() {
    double value = MAIN_CONTROLLER.getRawAxis(1);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }
  
  //For tankdrive control (unused)
  public double getMainRightJoyY() {
    double value = MAIN_CONTROLLER.getRawAxis(2);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }

  //Drivebase control
  public double getMainRightJoyX() {
    double value = MAIN_CONTROLLER.getRawAxis(3);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }

  //Drivebase throttle
  public double getMainRightTrigger() {
    double value = MAIN_CONTROLLER.getRawAxis(3);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }

  //Climber wheel actuation & outtaking
  public double getMainLeftTrigger() {
    double value = MAIN_CONTROLLER.getRawAxis(2);
    return Math.abs(value) > DEADZONE ? (Math.copySign(Math.abs(value) - DEADZONE, value) / (1.0 - DEADZONE)) : 0.0;
  }

  public boolean getMainBButtonPressed() {
    return MAIN_CONTROLLER.getRawButtonPressed(2);
  }

  public boolean getMainBButton() {
    return MAIN_CONTROLLER.getRawButton(2);
  }

  public boolean getMainLeftBumperPressed() {
    return MAIN_CONTROLLER.getRawButtonPressed(5);
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
    return MAIN_CONTROLLER.getRawButtonPressed(4);
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
	  return null;
  }
}