/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.ArmToPosition;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivebase;

public class RobotContainer {
  private static final double DEADZONE = 0.15;
  private Drivebase drive;
  private final double kS = 0.993;
  private final double kV = 0.00240; 
  private final double kA = 0.000212; 

  public Joystick main;
  // private JoystickButton XButton, YButton, AButton, BButton, RightBumper;
  // private double value;
  // private POVButton MainDPadDown, MainDPadUp;
  public Controller main;
  public Controller arm;
  public RobotContainer() { 

    drive = Robot.drivebase;
    drive.setDefaultCommand(new ArcadeDrive(drive));
    String trajectoryJSON = "PathWeaver/pathweaver.json" ;
    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }
    main = new Controller(Constants.CONTROLLER_MAIN);
    arm = new Controller(Constants.CONTROLLER_ARM);
    // YButton = new JoystickButton(main, 4);
    // AButton = new JoystickButton(main, 1);
    // XButton = new JoystickButton(main, 3);
    // BButton = new JoystickButton(main, 2);
    // ArmBButton = new JoystickButton(arm, 2);
    // ArmDPadUp = new POVButton(arm, 0);
    // ArmDPadDown = new POVButton(arm, 180);
    // RightBumper = new JoystickButton(main, 6);
    // MainDPadUp = new POVButton(main, 0);
    // MainDPadDown = new POVButton(main, 180);
    // ArmDPadRight = new POVButton(arm, 90);
    // ArmLeftBumper = new JoystickButton(arm, 5);
    // ArmRightBumper = new JoystickButton(arm, 6);

    // ArmDPadDown.whenPressed(new ArmToPosition(-78.5));
    // ArmDPadUp.whenPressed(new ArmToPosition(-17.38));
    // ArmDPadRight.whenPressed(new ArmToPosition(-42.3));

    //drivetrainOverride.whileActive(new ArcadeDrive());

    //AButton.whenPressed(new TurnToLine(5));
  }

  public Command getAutonomousCommand() {
    drive.resetHeading();
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kS, kV, kA), drive.getKinematics(), 12
        );

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    config.setKinematics(drive.getKinematics()).addConstraint(autoVoltageConstraint);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(
          new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
          new Pose2d(-2.0, -2.0, new Rotation2d(0.0))),
          config);
    RamseteCommand command = new RamseteCommand(trajectory, 
                  drive::getPose, 
                  new RamseteController(2.0, 0.8), 
                  drive.getFeedForward(), 
                  drive.getKinematics(), 
                  drive::getSpeeds, 
                  drive.getLeftPID(), 
                  drive.getRightPID(), 
                  drive::setOutput, 
                  drive);

    
    return command;
  }

public double getArmLeftJoyY() {
	return 0;
}

public double getArmRightTrigger() {
	return 0;
}
}