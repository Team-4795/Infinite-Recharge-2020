/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ArcadeDrive;

public class Drivebase extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  // private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kEncoderCountPerMeter = 18148 / (Units.feetToMeters(6) * Math.PI); // TODO: tune
  private static final double kP = 1; // TODO: tune PID
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;
  private static final double kEncoderCountPerFeet = 2050/1.57;

  public Encoder leftMotorEncoder;
  public Encoder rightMotorEncoder;


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
  //in theory should equal: (ENCODER_COUNTS_PER_REV * 12) / (Math.PI * WHEEL_DIAMETER_IN)

  private final TalonSRX leftMotor;
  private final TalonSRX leftMotorFollower;
  // private final VictorSPX leftMotorThree;
  private final TalonSRX rightMotor;
  private final TalonSRX rightMotorFollower;
  // private final VictorSPX rightMotorThree;
  // public final PIDController turnController;

  public boolean climbTime = false; 
  public boolean hasMoved;

  public Drivebase() {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    // m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // m_leftEncoder.reset();
    // m_rightEncoder.reset();


    // turnController = new PIDController(P, I, D, Robot.ahrs, this);
    // turnController.setInputRange(-180.0f, 180.0f);
    // turnController.setOutputRange(-0.6, 0.6);
    // turnController.setAbsoluteTolerance(Tolerance);
    // turnController.setContinuous();

    leftMotor = new TalonSRX(Constants.DRIVEBASE_LEFT_MAIN_TALON);
    leftMotorFollower = new TalonSRX(Constants.DRIVEBASE_LEFT_FOLLOWER_TALON);
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

    leftMotor.setInverted(true);
    leftMotorFollower.setInverted(true);
    leftMotorFollower.follow(leftMotor);

    rightMotor.setInverted(false);
    rightMotorFollower.setInverted(false);
    rightMotorFollower.follow(rightMotor);
    
    leftMotorEncoder = new Encoder(3,4); //2020, 2100, 2030
    rightMotorEncoder = new Encoder(7,8);

    // leftMotorEncoder.setReverseDirection(true);
    rightMotorEncoder.setReverseDirection(true);
    // leftMotor.setSelectedSensorPosition(0);
    // rightMotor.setSelectedSensorPosition(0);
  }

  public void setMotors(double left, double right) {
    SmartDashboard.putNumber("I LOVE SELF PLEASURE", leftMotorEncoder.getDistance());
    SmartDashboard.putNumber("I LOVE SELF PLEASURE3", rightMotorEncoder.getDistance());
    leftMotor.set(ControlMode.PercentOutput, left);
    rightMotor.set(ControlMode.PercentOutput, right);
  }

  public int getLeftEncoderCount() {
    return leftMotor.getSelectedSensorPosition();
  }
  public int getRightEncoderCount() {
    return rightMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderMeters() {
    return leftMotor.getSelectedSensorPosition() / kEncoderCountPerMeter;
  }
  public double getRightEncoderMeters() {
    return rightMotor.getSelectedSensorPosition() / kEncoderCountPerMeter;
  }

  // should give velocity in ft per second
  public double getLeftVelocity() {
    return leftMotor.getSelectedSensorVelocity() * 10 / kEncoderCountPerMeter;
  }

  public double getRightVelocity() {
    return rightMotor.getSelectedSensorVelocity() * 10 / kEncoderCountPerMeter;
  }
  
  public void driveFeet(double feet) {
    hasMoved = false;
    resetEncoders();
    // while (leftMotorEncoder.getDistance() < feet * kEncoderCountPerFeet) {
    //   setMotors(-0.2, -0.2);
    //   Robot.arm.goToPosition();
    // }
    // setMotors(0.0, 0.0);
    // Robot.arm.goToPosition();
    // Robot.arm.setRoller(-1);
    if (leftMotorEncoder.getDistance() < feet * kEncoderCountPerFeet) {
      setMotors(-0.215, -0.25);
      Robot.arm.goToPosition();
    } else {
      hasMoved = true;
      Robot.drivebase.setMotors(0.0, 0.0);
    }

    
  }
  
  public void resetEncoders() {
    // rightMotor.setSelectedSensorPosition(0);
    // leftMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    setDefaultCommand(new ArcadeDrive());
  }
}