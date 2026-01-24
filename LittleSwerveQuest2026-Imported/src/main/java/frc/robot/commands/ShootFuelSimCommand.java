package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.FuelSim;

/**
 * SIMULATION-ONLY command.
 * Spawns a FuelSim projectile representing one fired shot.
 */
public class ShootFuelSimCommand extends InstantCommand {

  public ShootFuelSimCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      ShooterSubsystem shooter) {

    super(() -> spawnShot(drive, turret, shooter));
  }

  private static void spawnShot(
      DriveSubsystem drive,
      TurretSubsystem turret,
      ShooterSubsystem shooter) {

    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fieldSpeeds = drive.getFieldSpeeds();

    // Absolute yaw = robot yaw + turret yaw
    Rotation2d launchYaw =
        robotPose.getRotation().plus(
            Rotation2d.fromDegrees(turret.getAngleDeg()));

    /* ---------------- Spawn position ---------------- */

    // Turret location on robot
    double tx = Constants.SimConstants.TURRET_X_M;
    double ty = Constants.SimConstants.TURRET_Y_M;
    double tz = Constants.SimConstants.TURRET_Z_M;

    // Muzzle offset
    double mx = Constants.SimConstants.MUZZLE_X_M;
    double my = Constants.SimConstants.MUZZLE_Y_M;
    double mz = Constants.SimConstants.MUZZLE_Z_M;

    double cosR = robotPose.getRotation().getCos();
    double sinR = robotPose.getRotation().getSin();

    // Turret offset rotated by robot yaw
    double turretX = tx * cosR - ty * sinR;
    double turretY = tx * sinR + ty * cosR;

    double cosL = launchYaw.getCos();
    double sinL = launchYaw.getSin();

    double muzzleX = mx * cosL - my * sinL;
    double muzzleY = mx * sinL + my * cosL;

    Translation3d startPosition =
        new Translation3d(
            robotPose.getX() + turretX + muzzleX,
            robotPose.getY() + turretY + muzzleY,
            tz + mz);

    /* ---------------- Launch velocity ---------------- */

    double shooterRpm = shooter.getVelocityRpm();

    double rps = shooterRpm / 60.0;
    double surfaceSpeed =
        2.0 * Math.PI *
        Constants.SimConstants.SHOOTER_WHEEL_RADIUS_M *
        rps;

    double exitSpeed =
        surfaceSpeed * Constants.SimConstants.SHOOTER_TO_BALL_EFFICIENCY;

    double vx =
        cosL * exitSpeed + fieldSpeeds.vxMetersPerSecond;
    double vy =
        sinL * exitSpeed + fieldSpeeds.vyMetersPerSecond;
    double vz =
        Constants.SimConstants.SHOT_UP_V_MPS;

    Translation3d launchVelocity =
        new Translation3d(vx, vy, vz);

    /* ---------------- Spawn fuel ---------------- */

    FuelSim.getInstance().spawnFuel(startPosition, launchVelocity);
  }
}
