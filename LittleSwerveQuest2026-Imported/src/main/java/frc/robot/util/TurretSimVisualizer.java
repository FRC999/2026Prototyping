package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSimVisualizer {
  private final DriveSubsystem drive;
  private final TurretSubsystem turret;

  private final StructPublisher<Pose3d> turretPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("Turret/TurretPose", Pose3d.struct)
          .publish();

  private final StructArrayPublisher<Translation3d> trajPub =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Turret/Trajectory", Translation3d.struct)
          .publish();

  public TurretSimVisualizer(DriveSubsystem drive, TurretSubsystem turret) {
    this.drive = drive;
    this.turret = turret;
  }

  /** Call this every sim tick. */
  public void update() {
    Pose2d robotPose = drive.getPose();

    // Turret yaw relative to robot (degrees -> radians)
    double turretYawRad = Math.toRadians(turret.getAngleDeg());

    // Where is your turret located on the robot? (set these constants)
    Translation3d turretOnRobot =
        new Translation3d(
            Constants.SimConstants.TURRET_X_M,
            Constants.SimConstants.TURRET_Y_M,
            Constants.SimConstants.TURRET_Z_M);

    // Convert robot pose (2D) into a 3D pose, then add turret yaw
    Pose3d base =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            turretOnRobot.getZ(),
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    Pose3d turretPose =
        new Pose3d(
            base.getTranslation().plus(new Translation3d(turretOnRobot.getX(), turretOnRobot.getY(), 0)),
            new Rotation3d(0, 0, robotPose.getRotation().getRadians() + turretYawRad));

    turretPosePub.set(turretPose);

    // Optional: publish a simple “fake” trajectory arc so you can see aim direction
    publishSimpleTrajectory(turretPose);
  }

  private void publishSimpleTrajectory(Pose3d turretPose) {
    // This is purely visualization (not physics-accurate).
    // You can later replace with Hammerheads’ ballistic calc if/when you want.

    Translation3d start = turretPose.getTranslation().plus(
        new Translation3d(Constants.SimConstants.MUZZLE_X_M,
                          Constants.SimConstants.MUZZLE_Y_M,
                          Constants.SimConstants.MUZZLE_Z_M));

    double yaw = turretPose.getRotation().getZ();

    // Pick any “display velocity” that looks reasonable
    double v = Constants.SimConstants.TRAJ_DISPLAY_V_MPS;

    Translation3d[] pts = new Translation3d[Constants.SimConstants.TRAJ_POINTS];
    for (int i = 0; i < pts.length; i++) {
      double t = i * Constants.SimConstants.TRAJ_DT_S;
      double x = start.getX() + Math.cos(yaw) * v * t;
      double y = start.getY() + Math.sin(yaw) * v * t;

      // little drop for looks
      double z = start.getZ() + (v * 0.15) * t - 0.5 * 9.81 * t * t * 0.15;

      pts[i] = new Translation3d(x, y, Math.max(0.0, z));
    }
    trajPub.set(pts);
  }
}
