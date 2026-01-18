// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveConstants {
    public final static int leftLeadCANID = 0;
    public final static int leftFollowCANID = 1;
    public final static int rightLeadCANID = 2;
    public final static int rightFollowCANID = 3;

    public final static boolean leftLeadInverted = false;
    public final static boolean leftFollowInverted = false;
    public final static boolean rightLeadInverted = false;
    public final static boolean rightFollowInverted = false;
  }

  public static class IntakeConstants{
    public final static int intakeMotorID = 0;
    public final static int launcherMotorID = 1;
  }
}
