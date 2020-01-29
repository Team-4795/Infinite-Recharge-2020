/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.frc.AHRS.BoardAxis;
// import com.kauailabs.navx.frc.AHRS.BoardYawAxis;

// import edu.wpi.first.wpilibj.PIDController;
// import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.SerialPort.Port;
// import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ArcadeDrive;

public class Drivebase extends SubsystemBase {
  
  public final TalonSRX leftMotor;
  private final VictorSPX leftMotorFollower;
  // private final VictorSPX leftMotorThree;
  public final TalonSRX rightMotor;
  private final TalonSRX rightMotorFollower;
  // private final VictorSPX rightMotorThree;
  // public final PIDController turnController;

  // private final static double P = -0.009;
  // private final static double I = 0.0;
  // private final static double D = -0.00;
  // private final static double Tolerance = 6.0f;

  // private final double kP = 0.008;
  // private final double kI = -0.00;
  // private final double kD = 0.0;
  // private final double kF = .065;
  // public final int allowableError = 100;

  // private final double WHEEL_DIAMETER_IN = 8.0;
  // private final int ENCODER_COUNTS_PER_REV = 4096;
  // public final double ENCODER_COUNTS_PER_FT = 4096;
  //in theory should equal: (ENCODER_COUNTS_PER_REV * 12) / (Math.PI * WHEEL_DIAMETER_IN)
  
  public Drivebase() {

    
    // turnController = new PIDController(P, I, D, Robot.ahrs, this);
    // turnController.setInputRange(-180.0f, 180.0f);
    // turnController.setOutputRange(-0.6, 0.6);
    // turnController.setAbsoluteTolerance(Tolerance);
    // turnController.setContinuous();

    leftMotor = new TalonSRX(Constants.DRIVEBASE_LEFT_MAIN_TALON);
    leftMotorFollower = new VictorSPX(Constants.DRIVEBASE_LEFT_FOLLOWER_VICTOR);
    // leftMotorThree = new VictorSPX(Constants.LEFT_MOTOR_THREE);
    rightMotor = new TalonSRX(Constants.DRIVEBASE_RIGHT_MAIN_TALON);
    rightMotorFollower = new TalonSRX(Constants.DRIVEBASE_RIGHT_FOLLOWER_TALON);
    // rightMotorThree = new VictorSPX(Constants.DRIVEBASE_RIGHT_VICTOR);

    // leftMotorOne.config_kP(0, kP);
    // leftMotorOne.config_kI(0, kI);
    // leftMotorOne.config_kD(0, kD);
    // leftMotorOne.config_kF(0, kF);
    // leftMotorOne.configAllowableClosedloopError(0, allowableError, 5000);
    // leftMotorOne.configMotionAcceleration(20000);
    // leftMotorOne.configMotionCruiseVelocity(15000);

    // rightMotorOne.config_kP(0, kP);
    // rightMotorOne.config_kI(0, kI);
    // rightMotorOne.config_kD(0, kD);
    // rightMotorOne.config_kF(0, kF);
    // rightMotorOne.configAllowableClosedloopError(0, allowableError, 5000);

    Robot.masterTalon(leftMotor);
    Robot.masterTalon(rightMotor);
    leftMotor.configOpenloopRamp(.4);
    rightMotor.configOpenloopRamp(.4);

    // Robot.initVictor(leftMotorTwo);
    // Robot.initVictor(leftMotorThree);
    // Robot.initVictor(rightMotorTwo);
    // Robot.initVictor(rightMotorThree);


    rightMotor.setInverted(true);
    rightMotorFollower.setInverted(true);
    // rightMotorThree.setInverted(true);

    leftMotorFollower.follow(leftMotor);
    // leftMotorThree.follow(leftMotorOne);

    rightMotorFollower.follow(rightMotor);
    // rightMotorThree.follow(rightMotorOne);
    
    // rightMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // leftMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // rightMotorOne.setSelectedSensorPosition(0);
    // leftMotorOne.setSelectedSensorPosition(0);
  }

  public void setMotors(double left, double right) {
    leftMotor.set(ControlMode.PercentOutput, left);
    rightMotor.set(ControlMode.PercentOutput, right);
  }

  // public double getLeftEncoderCount() {
  //   return leftMotorOne.getSelectedSensorPosition();
  // }

  // public double getRightEncoderCount() {
  //   return rightMotorOne.getSelectedSensorPosition();
  // }

  // public double getLeftEncoderFeet() {
  //   return leftMotorOne.getSelectedSensorPosition() / ENCODER_COUNTS_PER_FT;
  // }

  // public double getRightEncoderFeet() {
  //   return rightMotorOne.getSelectedSensorPosition() / ENCODER_COUNTS_PER_FT;
  // }

  // //should give velocity in ft per second
  // public double getLeftVelocity() {
  //   return leftMotorOne.getSelectedSensorVelocity() * 10 / ENCODER_COUNTS_PER_FT;
  // }

  // public double getRightVelocity() {
  //   return rightMotorOne.getSelectedSensorVelocity() * 10 / ENCODER_COUNTS_PER_FT;
  // }

  // public void TurnToAngle(double angle) {
  //   Robot.ahrs.reset();
  //   turnController.reset();
  //   turnController.setSetpoint(angle);
  //   turnController.enable();
    
  // }

  
  // public void driveFeet(double feet) {
  //   this.resetEncoders();
  //   leftMotorOne.set(ControlMode.MotionMagic, -feet * ENCODER_COUNTS_PER_FT);
  //   rightMotorOne.set(ControlMode.MotionMagic, -feet * ENCODER_COUNTS_PER_FT);
  // }
  
  // public void resetEncoders() {
  //   rightMotorOne.setSelectedSensorPosition(0);
  //   leftMotorOne.setSelectedSensorPosition(0);
  // }

  // @Override
  // public void pidWrite(double output) {
  //   setMotors(output, -output);
  // }

  @Override
  public void periodic() {
    setDefaultCommand(new ArcadeDrive());
  }
}