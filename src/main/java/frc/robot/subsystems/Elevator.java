/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /*
   * Creates a new Elevator.
   */

  private final CANPIDController elevatorController;
  private final CANSparkMax elevatorMotor;
  private final CANEncoder elevatorEncoder;

  // PID values for moving arm to position
  // private final static double P = 0.00055;
  private static double upPosLowered;
  private static double upPosNeutral;
  private static double upPosRaised;
  private static double downPosLowered;
  private static double downPosNeutral;
  private static double downPosRaised;
  private final static double kP = 0.00015;
  private final static double kI = 0.000001;
  private final static double kD = 0.0000000;
  private final static double kF = 0.0002;
  private final static double kTolerance = 5.0;

  public Elevator() {
    elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorController = new CANPIDController(elevatorMotor);
    elevatorEncoder = new CANEncoder(elevatorMotor);

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setOpenLoopRampRate(0.5);
    elevatorMotor.setClosedLoopRampRate(0.5);
    // ArmMotor.setParameter(ConfigParameter.kHardLimitRevEn, true);
    // ArmMotor.setParameter(ConstantParameter.kCanID,
    // RobotContainer.ARM_MOTOR.value);
    // ArmMotor.setInverted(true);
    elevatorController.setP(kP, 0);
    elevatorController.setI(kI, 0);
    elevatorController.setIZone(20, 0);
    elevatorController.setD(kD, 0);
    elevatorController.setFF(kF, 0);
    elevatorController.setOutputRange(-0.55, 0.55, 0);
    elevatorController.setSmartMotionMaxVelocity(4200, 0);
    elevatorController.setSmartMotionMaxAccel(2750, 0);
    elevatorController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    elevatorController.setSmartMotionAllowedClosedLoopError(1.0, 0);
  }

  public void setUpPosition(int type) {
    elevatorController.setReference(type > 1 ? (type > 2 ? upPosRaised : upPosNeutral) : upPosLowered, ControlType.kSmartMotion);
  }

  public void setDownPosition(int type) {
    elevatorController.setReference(type > 1 ? (type > 2 ? downPosRaised : downPosNeutral) : downPosLowered, ControlType.kSmartMotion);
  }

  public void climb(int type) {
    SmartDashboard.putNumber("Elevator Position (PID)", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Output", elevatorMotor.getAppliedOutput());
    setUpPosition(type);
    setDownPosition(type);
  }

  public void set(double speed) {
    elevatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position (PID)", elevatorEncoder.getPosition());
  }
}
