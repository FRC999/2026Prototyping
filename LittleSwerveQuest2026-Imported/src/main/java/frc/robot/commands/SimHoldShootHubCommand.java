package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.BallisticAimSolver;
import frc.robot.util.FuelSim;
import frc.robot.util.ShooterHoodSim;

/**
 * SIM ONLY: Hold-to-shoot command.
 * - Continuously aims at the hub (updates yaw/pitch/speed every loop)
 * - Fires repeatedly at a fixed cadence while scheduled
 * - Designed to be bound with whileTrue(button)
 */
public class SimHoldShootHubCommand extends Command {

  private final DriveSubsystem drive;
  private final TurretSubsystem turret;
  private final ShooterSubsystem shooter;
  private final ShooterHoodSim hoodSim;

  private final double shotsPerSecond;
  private final double minTimeBetweenShotsSec;

  private double nextShotTimeSec = 0.0;

  public SimHoldShootHubCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      ShooterHoodSim hoodSim,
      double shotsPerSecond) {

    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hoodSim = hoodSim;

    this.shotsPerSecond = shotsPerSecond;
    this.minTimeBetweenShotsSec = 1.0 / Math.max(0.1, shotsPerSecond);

    addRequirements(turret, shooter);
  }

  @Override
  public void initialize() {
    // SIM safety
    if (!RobotBase.isSimulation()) {
      nextShotTimeSec = Double.POSITIVE_INFINITY;
      return;
    }
    double now = Timer.getFPGATimestamp();
    nextShotTimeSec = now; // fire immediately on hold
  }

  @Override
  public void execute() {
    if (!RobotBase.isSimulation()) return;

    // ---------------- Read state ----------------
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds vField = drive.getFieldSpeeds();
    Translation3d vRobotField = new Translation3d(
        vField.vxMetersPerSecond,
        vField.vyMetersPerSecond,
        0.0);

    // Target hub (FIELD frame)
    Translation3d hub = new Translation3d(
        Constants.SimConstants.HUB_CENTER_X_M,
        Constants.SimConstants.HUB_CENTER_Y_M,
        Constants.SimConstants.HUB_CENTER_Z_M);

    // ---------------- Ballistic solve ----------------
    // First yaw guess points at hub
    double dx = hub.getX() - robotPose.getX();
    double dy = hub.getY() - robotPose.getY();
    double yawFieldGuess = Math.atan2(dy, dx);

    // Two-pass refinement for muzzle offset
    Translation3d muzzleStart = computeMuzzleStartField(robotPose, yawFieldGuess);
    BallisticAimSolver.Solution sol = BallisticAimSolver.solve(muzzleStart, hub, vRobotField);

    muzzleStart = computeMuzzleStartField(robotPose, sol.yawFieldRad);
    sol = BallisticAimSolver.solve(muzzleStart, hub, vRobotField);

    // ---------------- Command turret + shooter ----------------
    double turretYawRobotDeg = sol.turretYawRobotDeg(robotPose.getRotation().getRadians());
    turret.goToAngleDeg(turretYawRobotDeg);

    hoodSim.setPitchDeg(sol.pitchDeg());
    shooter.setTargetRpm(sol.wheelRpm);

    // ---------------- Fire at cadence ----------------
    double now = Timer.getFPGATimestamp();
    if (now >= nextShotTimeSec) {
      FuelSim.getInstance().spawnFuel(muzzleStart, sol.launchVelocityField);

      nextShotTimeSec = now + minTimeBetweenShotsSec;

      // Optional debug (comment out if noisy)
      System.out.println(
          "[SIM AUTO SHOT] yawRobotDeg=" + turretYawRobotDeg
              + " pitchDeg=" + sol.pitchDeg()
              + " wheelRpm=" + sol.wheelRpm
              + " vLaunch=" + sol.launchVelocityField);
    }
  }

  @Override
  public boolean isFinished() {
    // Never finishes on its own; whileTrue() will cancel on button release.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop “auto fire” timing so a new hold starts clean
    nextShotTimeSec = Double.POSITIVE_INFINITY;
  }

  private static Translation3d computeMuzzleStartField(Pose2d robotPose, double yawFieldRad) {
    double tx = Constants.SimConstants.TURRET_X_M;
    double ty = Constants.SimConstants.TURRET_Y_M;
    double tz = Constants.SimConstants.TURRET_Z_M;

    double mx = Constants.SimConstants.MUZZLE_X_M;
    double my = Constants.SimConstants.MUZZLE_Y_M;
    double mz = Constants.SimConstants.MUZZLE_Z_M;

    Rotation2d robotYaw = robotPose.getRotation();
    double cosR = robotYaw.getCos();
    double sinR = robotYaw.getSin();

    double turretFieldX = tx * cosR - ty * sinR;
    double turretFieldY = tx * sinR + ty * cosR;

    double cosY = Math.cos(yawFieldRad);
    double sinY = Math.sin(yawFieldRad);

    double muzzleFieldX = mx * cosY - my * sinY;
    double muzzleFieldY = mx * sinY + my * cosY;

    return new Translation3d(
        robotPose.getX() + turretFieldX + muzzleFieldX,
        robotPose.getY() + turretFieldY + muzzleFieldY,
        tz + mz);
  }
}
