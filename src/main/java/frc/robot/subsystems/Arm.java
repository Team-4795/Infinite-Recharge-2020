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

public class Arm extends SubsystemBase {

    private final CANSparkMax ArmMotor;

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
    private boolean gucci;

    public Arm() {

        ArmMotor = new CANSparkMax(Constants.ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        armEnc = new CANEncoder(ArmMotor);
        topLimit = new CANDigitalInput(ArmMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);

        ArmMotor.setIdleMode(IdleMode.kBrake);
        ArmMotor.setOpenLoopRampRate(0.5);
        ArmMotor.setClosedLoopRampRate(0.5);
        // ArmMotor.setParameter(ConfigParameter.kHardLimitRevEn, true);
        // ArmMotor.setParameter(ConstantParameter.kCanID, RobotContainer.ARM_MOTOR.value);
        // ArmMotor.setInverted(true);

  }
  
  }

  public double getPos() {
    return armEnc.getPosition();
  }

  public Boolean getTopLimit() {
    return topLimit.get();
  }

  public void resetEnc() {
    armEnc.setPosition(0.0);
  }

  public void actuate(final double output) {
        
    }

  public void intake(double upPos) {
    armController.setReference(upPos, ControlType.kSmartMotion);
  }

  public void outtake(double downPos) {
    armController.setReference(downPos, ControlType.kSmartMotion);
  }

    /*
     * public void setPosition(double position) {
     * armController.setReference(position, ControlType.kPosition); }
     */

    

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualArmControl());
  }
}