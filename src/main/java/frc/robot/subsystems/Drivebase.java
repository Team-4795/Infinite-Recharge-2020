
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.BoardAxis;
import com.kauailabs.navx.frc.AHRS.BoardYawAxis;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
//import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.frc.AHRS.BoardAxis;
// import com.kauailabs.navx.frc.AHRS.BoardYawAxis;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class Drivebase extends SubsystemBase {
  public final TalonSRX leftMotor;
  private final TalonSRX leftMotorFollower;
  public final TalonSRX rightMotor;
  private final TalonSRX rightMotorFollower;
  public DifferentialDriveKinematics kinematics; 
  public DifferentialDriveOdometry odometry;
  public AHRS gyro;
  private Pose2d pose; 

  private final double kS = 0.993;
  private final double kV = 0.00239; 
  private final double kA = 0.000212; 
  public SimpleMotorFeedforward feedforward;
  public PIDController pidRight;
  public PIDController pidLeft;
  private final double kP = 0.000908;
  private final double kI = 0.0;
  private final double kD = 0.0;
  // private final VictorSPX rightMotorThree;
  // public final PIDController turnController;
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final int kEncoderCountPerRevolution = 18148;
  private static final double kEncoderCountPerMeter = kEncoderCountPerRevolution / (Units.inchesToMeters(6) * Math.PI);

  // private final static double P = -0.009;
  // private final static double I = 0.0;
  // private final static double D = -0.00;
  // private final static double Tolerance = 6.0f;

  // private final double kP = 0.008;
  // private final double kI = -0.00;
  // private final double kD = 0.0;
  // private final double kF = .065;
  // public final int allowableError = 100;

  private final double kWheelDiameter = Units.inchesToMeters(6.0);
  // private final int ENCODER_COUNTS_PER_REV = 4096;
  private final double kEncoderCountPerRevolution = 18148;
  // private final double ENCODER_COUNTS_PER_METER = kEncoderCountPerRevolution / (kWheelDiameter * Math.PI);
  // in theory should equal: (ENCODER_COUNTS_PER_REV * 12) / (Math.PI * WHEEL_DIAMETER_IN)
  
  public Drivebase() {
    gyro = new AHRS(SPI.Port.kMXP);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
    odometry = new DifferentialDriveOdometry(getHeading());
    feedforward = new SimpleMotorFeedforward(kS, kV, kA); 
    pidRight = new PIDController(kP, kI, kD);
    pidLeft = new PIDController(kP, kI, kD);

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
    rightMotor = new TalonSRX(Constants.DRIVEBASE_RIGHT_MAIN_TALON);
    rightMotorFollower = new TalonSRX(Constants.DRIVEBASE_RIGHT_FOLLOWER_TALON);

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
    // Robot.initVictor(rightMotorTwo);

    leftMotor.setInverted(false);
    leftMotorFollower.setInverted(false);
    leftMotorFollower.follow(leftMotor);

    rightMotor.setInverted(true);
    rightMotorFollower.setInverted(true);
    rightMotorFollower.follow(rightMotor);
    
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    rightMotor.setSelectedSensorPosition(0);
    leftMotor.setSelectedSensorPosition(0);
  }
  public void resetHeading() {
    gyro.zeroYaw();
    gyro.setAngleAdjustment(-gyro.getAngle());
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
  public void setMotors(double left, double right) {
    leftMotor.set(ControlMode.PercentOutput, left);
    rightMotor.set(ControlMode.PercentOutput, right);
  }

  public PIDController getLeftPID() {
    return pidLeft;
  }
  public PIDController getRightPID() {
    return pidRight;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
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

  // Should give velocity in meters per second
  public double getLeftVelocity() {
    // 10x multiplier because the returned value is the distance in 100ms
    return leftMotor.getSelectedSensorVelocity() * 10 / kEncoderCountPerMeter;
  }

  public double getRightVelocity() {
    // 10x multiplier because the returned value is the distance in 100ms
    return rightMotor.getSelectedSensorVelocity() * 10 / kEncoderCountPerMeter;
  }

  public void setOutput(double leftVoltage, double rightVoltage) {
    leftMotor.set(ControlMode.PercentOutput, leftVoltage / 5);
    rightMotor.set(ControlMode.PercentOutput, rightVoltage / 5);
  }
  
  // public void driveFeet(double feet) {
  //   this.resetEncoders();
  //   leftMotorOne.set(ControlMode.MotionMagic, -feet * ENCODER_COUNTS_PER_FT);
  //   rightMotorOne.set(ControlMode.MotionMagic, -feet * ENCODER_COUNTS_PER_FT);
  // }
  
  // public void resetEncoders() {
  //   rightMotorOne.setSelectedSensorPosition(0);
  //   leftMotorOne.setSelectedSensorPosition(0);
  // }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftVelocity(), // 7.29 * Math.PI * kWheelDiameter / 60,
      getRightVelocity()); // 7.29 * Math.PI * kWheelDiameter / 60);
  }
  
  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLeftEncoderMeters(), getRightEncoderMeters());
    SmartDashboard.putNumber("heading (deg)", gyro.getAngle());
    SmartDashboard.putNumber("encoder left (m)", getLeftEncoderMeters());
    SmartDashboard.putNumber("raw sensor velocity", leftMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("raw sensor position", leftMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("encoder right (m)", getRightEncoderMeters());
    SmartDashboard.putNumber("odometry x (m)", pose.getTranslation().getX());
    SmartDashboard.putNumber("odometry y (m)", pose.getTranslation().getY());
    SmartDashboard.putNumber("odometry angle (deg)", pose.getRotation().getDegrees());
  }
}
