package frc.robot.OdometryUpdates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class OdometryConstants {

  /** Alliance-specific initial yaws (field-centric) for seeding MegaTag2. */
  public static final Rotation2d INITIAL_YAW_RED  = Rotation2d.fromDegrees(  0);
  public static final Rotation2d INITIAL_YAW_BLUE = Rotation2d.fromDegrees(180);
  public static final Rotation2d TELEOP_YAW_RED = Rotation2d.fromDegrees(180);
  public static final Rotation2d TELEOP_YAW_BLUE = Rotation2d.fromDegrees(0);

  public static Rotation2d initialYawForAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red ? INITIAL_YAW_RED : INITIAL_YAW_BLUE;
  }

  public static Rotation2d teleopYawForAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red ? TELEOP_YAW_RED : TELEOP_YAW_BLUE;
  }

}
