/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Controller Mappings
  public final static int CONTROLLER_MAIN = 0;
  public final static int CONTROLLER_ARM = 1;

  // CAN Motor Controller Mappings
  public final static int DRIVEBASE_LEFT_MAIN_TALON = 2;
  public final static int DRIVEBASE_LEFT_FOLLOWER_TALON = 3;
  public final static int DRIVEBASE_RIGHT_MAIN_TALON = 4;
  public final static int DRIVEBASE_RIGHT_FOLLOWER_TALON = 5;
  public final static int ARM_MOTOR = 10;
  public final static int ARM_SPINNER = 8;
  public final static int ARM_ROLLER = 9;
  public final static int ELEVATOR_MOTOR_SPARK = 20;
}