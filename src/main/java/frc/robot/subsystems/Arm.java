/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.ManualArmControl;

public class Arm extends SubsystemBase {

  private final CANSparkMax armMotor;

  private final VictorSPX roller;
  // soft limit for arm in encoder ticks
  private final static double kLowerLimit = -77.69;

  private final CANPIDController armController;

  // PID values for moving arm to position
  // private final static double P = 0.00055;
  private final static double kP = 0.00015;
  private final static double kI = 0.000001;
  private final static double kD = 0.0000000;
  private final static double kF = 0.0002;
  private final static double kTolerance = 5.0;

  private final int kEncoderRotsPerSpinner = 0;

  private final CANEncoder armEncoder;
  private final CANDigitalInput topLimit;

  private double up;
  private double down;
  private boolean isRollerBackward; 

  public Arm() {
    isRollerBackward = true; 
    armMotor = new CANSparkMax(Constants.ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    armController = new CANPIDController(armMotor);
    armEncoder = new CANEncoder(armMotor);
    topLimit = new CANDigitalInput(armMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
    roller = new VictorSPX(Constants.ARM_ROLLER);

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setOpenLoopRampRate(0.5);
    armMotor.setClosedLoopRampRate(0.5);
    // armMotor.setParameter(ConfigParameter.kHardLimitRevEn, true);
    // armMotor.setParameter(ConstantParameter.kCanID, RobotContainer.ARM_MOTOR.value);
    // armMotor.setInverted(true);
    armController.setP(kP, 0);
    armController.setI(kI, 0);
    armController.setIZone(20, 0);
    armController.setD(kD, 0);
    armController.setFF(kF, 0);
    armController.setOutputRange(-0.55, 0.55, 0);
    armController.setSmartMotionMaxVelocity(4200, 0);
    armController.setSmartMotionMaxAccel(2750, 0); 
    armController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    armController.setSmartMotionAllowedClosedLoopError(1.0, 0); 

    roller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    roller.setSelectedSensorPosition(0);
  }
  public void setArm(double speed) {
    armMotor.set(speed);
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

  public void actuate(double speed) {
    armController.setReference(speed, ControlType.kVoltage); // FIXME: * 12 for voltage?
  }

  public void intake() {
    armController.setReference(down, ControlType.kSmartMotion);
  }
  public void outtake() {
    armController.setReference(up, ControlType.kSmartMotion);
  }

  public void setRoller(double speed) {
    // roller.set(ControlMode.Position, 4 * kEncoderRotsPerSpinner * mult);
    roller.set(ControlMode.PercentOutput, speed);
  }

  public void setPosition(double position) {
    armController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    setDefaultCommand(new ManualArmControl());
  }
}
