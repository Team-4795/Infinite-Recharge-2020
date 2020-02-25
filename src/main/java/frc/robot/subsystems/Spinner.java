/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.ColorSensor;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ColorSensor.ColorData;



// When looking down from the sky and spinning motor clockwise
// the wheel spins counter clockwise and the colors iterate from
// blue/cyan to green to red to yellow
// RGB: 0?1      010     100     110

public class Spinner extends SubsystemBase {

  public enum SpinnerColor {
    BLUE(0),
    GREEN(1),
    RED(2),
    YELLOW(3),
    NONE(-1);

    public final int colorId;
    private SpinnerColor(int colorIdTo) {
      colorId = colorIdTo;
    }
  };

  private ColorSensor cs = ColorSensor.getInstanceMXP();
  private TalonSRX motor;
  private SpinnerColor current = SpinnerColor.NONE;
  private SpinnerColor target = SpinnerColor.NONE;
  private int edgesRequired = 0;
  
  public Spinner() {
    motor = new TalonSRX(Constants.SPINNER_TALON);
    Robot.masterTalon(motor);
  }

  public void actuate(double speed) {
    if (speed < 0) speed = 0;
    motor.set(ControlMode.PercentOutput, speed);
  }
  
  public int getEncoderCount() {
      return motor.getSelectedSensorPosition();
  }

  public void turnCycles(int cycles) {
    current = SpinnerColor.NONE;
    target = SpinnerColor.NONE;
    edgesRequired = cycles * 8;
  }
  public void turnTo(SpinnerColor targetTo) {
    current = SpinnerColor.NONE;
    target = targetTo;
    edgesRequired = 0;
  }
  public void disable() {
    current = SpinnerColor.NONE;
    target = SpinnerColor.NONE;
    edgesRequired = 0;
  }

  @Override
  public void periodic() {
    ColorData color = cs.getColor();

    double threshold = Math.max(Math.max(color.red, color.green), color.blue) * 0.5;
    boolean red = color.red >= threshold;
    boolean green = color.green >= threshold;
    boolean blue = color.blue >= threshold;

    SpinnerColor detected =
      blue && red && green ? SpinnerColor.NONE :
      blue ? SpinnerColor.BLUE :
      red && green ? SpinnerColor.YELLOW :
      red ? SpinnerColor.RED :
      green ? SpinnerColor.GREEN :
      SpinnerColor.NONE;

    if (current == SpinnerColor.NONE) {
      current = detected;
    } else if (current != detected && detected != SpinnerColor.NONE) {
      current = detected;
      if (edgesRequired > 0)
        edgesRequired -= 1;
    }

    if (edgesRequired > 0 || current != target) {
      actuate(0.5);
    } else {
      actuate(0.0);
    }
  }
}
