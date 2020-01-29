/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // //CAN Motor Controller Mappings
  // LEFT_MOTOR_ONE(1),
  // LEFT_MOTOR_TWO(2),
  // // LEFT_MOTOR_THREE(3),
  // RIGHT_MOTOR_ONE(4),
  // RIGHT_MOTOR_TWO(5),
  // // RIGHT_MOTOR_THREE(6),
  // ARM_MOTOR(7),
  // INTAKE_MOTOR(11),
  // CLIMBER_WHEELS(12),
  // HATCH_MOTOR(8),
  // // CLIMBER_MOTOR(20),

  // Controller Mappings
  public final static int MAIN_CONTROLLER = 0;

  // CAN Motor Controller Mappings
  public final static int DRIVEBASE_LEFT_FOLLOWER_VICTOR = 2;
  public final static int DRIVEBASE_LEFT_MAIN_TALON = 3;
  public final static int DRIVEBASE_RIGHT_FOLLOWER_TALON = 4;
  public final static int DRIVEBASE_RIGHT_MAIN_TALON = 5;
}