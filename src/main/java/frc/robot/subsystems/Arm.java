/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ManualArmControl;
import java.time.Duration;;

public class Arm extends SubsystemBase {

    private final CANSparkMax ArmMotor;

    private final TalonSRX intake;
    // soft limit for arm in encoder ticks
    private final double lowerLimit = -77.69;

    private final CANPIDController armController;

    // PIDF values for balancing when climbing
    private static double Pb = 0.0225;
    private static double Ib = 0.0;
    private static double Db = 0.00;

    // PID values for moving arm to position
    // private static double P = 0.00055;
    private static double P = 0.00015;
    private static double I = 0.000001;
    private static double D = 0.0000000;
    private static double F = 0.0002;

    private static double middle; 

    private final static double Tolerance = 5.0f;
    private final PIDController armBalancer;
    private final CANEncoder armEnc;
    private final CANDigitalInput topLimit;

    private double up;
    private double down;

    private boolean gucci;

    public Arm() {

        ArmMotor = new CANSparkMax(Constants.ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        armBalancer = new PIDController(P, I, D);
        armController = new CANPIDController(ArmMotor);
        armEnc = new CANEncoder(ArmMotor);
        topLimit = new CANDigitalInput(ArmMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
        intake = new TalonSRX(Constants.ARM_INTAKE);

        ArmMotor.setIdleMode(IdleMode.kBrake);
        ArmMotor.setOpenLoopRampRate(0.5);
        ArmMotor.setClosedLoopRampRate(0.5);
        // ArmMotor.setParameter(ConfigParameter.kHardLimitRevEn, true);
        // ArmMotor.setParameter(ConstantParameter.kCanID, RobotContainer.ARM_MOTOR.value);
        // ArmMotor.setInverted(true);
        armController.setP(P, 0);
        armController.setI(I, 0);
        armController.setIZone(20, 0);
        armController.setD(D, 0);
        armController.setFF(F, 0);
        armController.setOutputRange(-0.55, 0.55, 0);
        armController.setSmartMotionMaxVelocity(4200, 0);
        armController.setSmartMotionMaxAccel(2750, 0); 
        armController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        armController.setSmartMotionAllowedClosedLoopError(1.0, 0); 
      
    
  }
  public void setArm(double speed) {
    ArmMotor.set(speed);
  }
  public void setIntake(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  
  }
  public double getPos() {
    return armEnc.getPosition();
  }

  public double getVel() {
    return armEnc.getVelocity();
  }

  public Boolean getTopLimit() {
    return topLimit.get();
  }

  public void resetEnc() {
    armEnc.setPosition(0.0);
  }

  // public void actuate(final double output) {
        
  //   }

  public void intake() {
    double downPos = this.down;
    armController.setReference(downPos, ControlType.kSmartMotion);
  }

  public void outtake() {
    double upPos = this.up;
    armController.setReference(upPos, ControlType.kSmartMotion);
  }

  public void setMotorWithTicks(boolean isNeg) {
    double  mult = 1.0;
    if (isNeg) {
      mult *= -1.0;
    }
    int ticks = 0;
    while (ticks < 100) {
      this.setIntake(0.5*mult);
      ticks += 1;
    }
    mult = 1.0;
  }

  public void ballPickUp() {
    this.intake();
    this.setMotorWithTicks(true);
    this.outtake();
    this.setMotorWithTicks(false);
    
  }

    /*
     * public void setPosition(double position) {
     * armController.setReference(position, ControlType.kPosition); }
     */

    

  @Override
  public void periodic() {
    setDefaultCommand(new ManualArmControl());
  }
}
