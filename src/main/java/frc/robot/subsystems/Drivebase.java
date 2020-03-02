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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.frc.AHRS.BoardAxis;
// import com.kauailabs.navx.frc.AHRS.BoardYawAxis;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ArcadeDrive;

public class Drivebase extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final int kEncoderCountPerRevolution = 18148;
  private static final double kEncoderCountPerMeter = kEncoderCountPerRevolution / (Units.inchesToMeters(6) * Math.PI);

  private static final double kP = 1; // TODO: tune PID
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;

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

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final PIDController m_leftPIDController = new PIDController(kP, kI, kD, kF);
  private final PIDController m_rightPIDController = new PIDController(kP, kI, kD, kF);

  private final DifferentialDriveKinematics m_kinematics
      = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  public Drivebase() {
    m_gyro.reset();

    m_odometry = new DifferentialDriveOdometry(getAngle());

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

    leftMotor.setInverted(false);
    leftMotorFollower.setInverted(false);
    leftMotorFollower.follow(leftMotor);

    rightMotor.setInverted(true);
    rightMotorFollower.setInverted(true);
    rightMotorFollower.follow(rightMotor);
    
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);
  }

  public void setMotors(double left, double right) {
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

  /**
   * Constructs a differential drive object.
   * Sets the encoder distance per pulse and resets the gyro.
   */

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    double leftOutput = m_leftPIDController.calculate(
      leftMotor.getSelectedSensorVelocity() / kEncoderCountPerMeter,
        speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(
      rightMotor.getSelectedSensorVelocity() / kEncoderCountPerMeter,
        speeds.rightMetersPerSecond);
    leftMotor.set(ControlMode.PercentOutput, leftOutput + leftFeedforward);
    rightMotor.set(ControlMode.PercentOutput, rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /**
   * Updates the field-relative position.
   */
  public void updateOdometry() {
    m_odometry.update(getAngle(), getLeftEncoderMeters(), getRightEncoderMeters());
  }

  @Override
  public void periodic() {
    setDefaultCommand(new ArcadeDrive());
  }
}