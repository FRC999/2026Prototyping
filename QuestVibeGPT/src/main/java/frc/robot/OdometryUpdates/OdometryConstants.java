package frc.robot.OdometryUpdates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class OdometryConstants {

  /** Alliance-specific initial yaws (field-centric) for seeding MegaTag2. */
  public static final Rotation2d INITIAL_YAW_RED  = Rotation2d.fromDegrees(  0);
  public static final Rotation2d INITIAL_YAW_BLUE = Rotation2d.fromDegrees(180);

  public static Rotation2d initialYawForAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red ? INITIAL_YAW_RED : INITIAL_YAW_BLUE;
  }

  /** Limelight IMU mode for MT2: 0=external only, 1=external seeds LL IMU, 2=internal. */
  public static final int LL_IMU_MODE = 1;

}
