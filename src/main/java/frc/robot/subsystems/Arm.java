/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
// import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.ManualArmControl;

public class Arm extends SubsystemBase {

  private final CANSparkMax armMotor;

  public static final Servo servo = new Servo(Constants.SErVo_ChAnIeL);

  private final VictorSPX roller;
  // soft limit for arm in encoder ticks

  private final CANPIDController armController;

  // PID values for moving arm to position
  // private final static double P = 0.00055;
  private final static double kP = 0.035;
  private final static double kI = 0.0;
  private final static double kD = 0.0;
  // private final static double kF = 0.002;

  private static double kP1 = 0.005;
  private static double kI1 = 0;
  private static double kD1 = 0;
  // private static double kF1 = 1;

  // private final static double kTolerance = 5.0;

  private final CANEncoder armEncoder;
  private CANDigitalInput topLimit;

  private double up = 6;
  private double down = 30;
  private double reference = up;
  public double armPositionMode = 0;
  private double tolerance = 5;
  public boolean pidEnabled = false;

  public Arm() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    armController = new CANPIDController(armMotor);
    armEncoder = new CANEncoder(armMotor);
    // topLimit = new CANDigitalInput(armMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyClosed);
    topLimit = armMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen); // Reverse works in wrong direction with normally closed
    roller = new VictorSPX(Constants.ARM_ROLLER);
    // servo.setAngle(0);
    // SmartDashboard.putNumber("SERVO", servo.getAngle()););
    
    // servo.setAngle(0);

    SmartDashboard.putNumber("peen", kP1);
    SmartDashboard.putNumber("iiiii", kI1);
    SmartDashboard.putNumber("ddddd", kD1);
    armMotor.setIdleMode(IdleMode.kBrake);
    // armMotor.setOpenLoopRampRate(0.3);
    // armMotor.setClosedLoopRampRate(0.1);
    // ArmMotor.setParameter(ConfigParameter.kHardLimitRevEn, true);
    // ArmMotor.setParameter(ConstantParameter.kCanID, RobotContainer.ARM_MOTOR.value);
    // ArmMotor.setInverted(true);
    armController.setP(kP, 0);
    armController.setI(kI, 0);
    armController.setIZone(20, 0);
    armController.setD(kD, 0);
    // armController.setFF(kF, 0);
    // armController.setOutputRange(-0.8, 0.8, 0);
    // armController.setSmartMotionMaxVelocity(4200, 0);
    // armController.setSmartMotionMaxAccel(4000, 0); 
    // armController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // armController.setSmartMotionAllowedClosedLoopError(2.0, 0); 

    armEncoder.setPosition(0);
  }

  public void setArm(double speed, boolean override) {
    setServo(armEncoder.getPosition() * 1.15);
    if (!override && armEncoder.getPosition() < 2 && speed < 0) speed = 0;
    // if (armEncoder.getPosition() > 15 && speed > 0) speed *= 0.25;
    if (!override && armEncoder.getPosition() > 18 && speed > 0) speed = 0;
    // if (speed > 0) { // down
    //   speed = (1 - (armEncoder.getPosition() / 30)) * 0.1;
    //   //speed = 0.05*speed*Math.cos(armEncoder.getPosition() / 30 * 90) + 0.05;
    // } else if (speed < 0) { // up
    //   speed = ((armEncoder.getPosition() / 30) - 1) * 0.1;
    //   //speed = 0.3*speed*Math.sin(armEncoder.getPosition() / 30 * 90) - 0.05;
    // }
    // SmartDashboard.putNumber("Fuck aidan's cock", armEncoder.getPosition());
    armMotor.set(speed);
  }

  public void setVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  public double getPos() {
    return armEncoder.getPosition();
  }

  public double getVel() {
    return armEncoder.getVelocity();
  }

  public Boolean getTopLimit() {
    return topLimit.get();
  }

  public void resetEnc() {
    armEncoder.setPosition(0.0);
  }

  // public void goToIntake() {
  //   armController.setP(kP);
  //   armController.setI(kI);
  //   armController.setD(kD);
  //   reference = up;
  //   // armController.setFF(kF);
  //   //armController.setReference(down, ControlType.kPosition);
  //   //SmartDashboard.putBoolean("activated", true);
  // }

  // public void goToOuttake() {
  //   armController.setP(kP1);
  //   armController.setI(kI1);
  //   armController.setD(kD1);
  //   reference = down;
  //   // armController.setFF(kF1);
  //   if(SmartDashboard.getNumber("peen", 0) != kP1) {
  //     kP1 = SmartDashboard.getNumber("peen", 0);
  //     SmartDashboard.putNumber("peeeeeen", kP1);
  //     armController.setP(kP1);
  //   }
  //   if(SmartDashboard.getNumber("iiiii", 0) != kI1) {
  //     kI1 = SmartDashboard.getNumber("iiiii", 0);
  //     armController.setI(kI1);
  //   }
  //   if(SmartDashboard.getNumber("ddddd", 0) != kD1) {
  //     kD1 = SmartDashboard.getNumber("ddddd", 0);
  //     armController.setP(kD1);
  //   }
  //   //armController.setReference(up, ControlType.kPosition);
  // }

  public void setPosition(int position) {
    armPositionMode = position;
    if (position == 0) {
      armController.setP(kP);
      armController.setI(kI);
      armController.setD(kD);
      reference = up;
    } else if (position == 1) {
      armController.setP(kP1);
      armController.setI(kI1);
      armController.setD(kD1);
      reference = down;
    } else if (position == 2) {
      armController.setP(0.05);
      armController.setI(kI);
      armController.setD(kD);
      reference = 0;
    }
  }

  public void setRoller(double speed) {
    roller.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("sped", speed);
  }

  // public void goToPosition(double position) {
  //   if(armPositionMode == 1 && Math.abs(armEncoder.getPosition()) - down < tolerance) return;
  //   SmartDashboard.putNumber("Fuck aidan's cock", Math.abs(armEncoder.getPosition()) - down);
  //   armController.setReference(position, ControlType.kPosition);
  // }

  public void setServo(double angle) {
    servo.setAngle(angle + 80);
  }

  public void goToPosition() {
    // if(SmartDashboard.getNumber("peen", 0) != kP1) {
    //   kP1 = SmartDashboard.getNumber("peen", 0);
    //   SmartDashboard.putNumber("peeeeeen", kP1);
    //   armController.setP(kP1);
    // }
    // if(SmartDashboard.getNumber("iiiii", 0) != kI1) {
    //   kI1 = SmartDashboard.getNumber("iiiii", 0);
    //   armController.setI(kI1);
    // }
    // if(SmartDashboard.getNumber("ddddd", 0) != kD1) {
    //   kD1 = SmartDashboard.getNumber("ddddd", 0);
    //   armController.setP(kD1);
    // }
    // SmartDashboard.putNumber("reference", reference);
    // SmartDashboard.putBoolean("achyutaIsHot", pidEnabled);
    if(armPositionMode == 1 && Math.abs(Math.abs(armEncoder.getPosition()) - down) < tolerance) return;
    SmartDashboard.putNumber("Fuck aidan's cock", Math.abs(Math.abs(armEncoder.getPosition()) - down));
    armController.setReference(reference, ControlType.kPosition);
    setServo(armEncoder.getPosition() * 1.15);
  }

  @Override
  public void periodic() {
    setDefaultCommand(new ManualArmControl());
  }
}
