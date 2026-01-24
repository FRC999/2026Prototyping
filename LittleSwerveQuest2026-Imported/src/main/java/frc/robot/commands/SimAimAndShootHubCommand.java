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
 * SIM-ONLY:
 * - Continuously aims turret + hood at the hub using ballistic math (with
 * robot-motion compensation)
 * - Fires exactly once per scheduling (button press)
 * - Holds aim briefly after firing so the turret stays pointed at the hub
 * during the shot
 */
public class SimAimAndShootHubCommand extends Command {

  private final DriveSubsystem drive;
  private final TurretSubsystem turret;
  private final ShooterSubsystem shooter;
  private final ShooterHoodSim hoodSim;

  private final double holdAfterFireSec;

  private boolean fired = false;
  private double fireTimeSec = 0.0;

  public SimAimAndShootHubCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      ShooterHoodSim hoodSim,
      double holdAfterFireSec) {

    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hoodSim = hoodSim;
    this.holdAfterFireSec = holdAfterFireSec;

    addRequirements(turret, shooter);
  }

  @Override
  public void initialize() {
    fired = false;
    fireTimeSec = 0.0;

    // Hard safety: SIM only
    if (!RobotBase.isSimulation()) {
      // Immediately finish without doing anything on real robot
      fired = true;
      fireTimeSec = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void execute() {
    // If we're on real robot, do nothing
    if (!RobotBase.isSimulation())
      return;

    Pose2d robotPose = drive.getPose();
    ChassisSpeeds vField = drive.getFieldSpeeds();

    Translation3d vRobotField = new Translation3d(
        vField.vxMetersPerSecond,
        vField.vyMetersPerSecond,
        0.0);

    // Hub target (FIELD frame)
    Translation3d hub = new Translation3d(
        Constants.SimConstants.HUB_CENTER_X_M,
        Constants.SimConstants.HUB_CENTER_Y_M,
        Constants.SimConstants.HUB_CENTER_Z_M);

    // First yaw guess toward hub
    double dx = hub.getX() - robotPose.getX();
    double dy = hub.getY() - robotPose.getY();
    double yawFieldGuess = Math.atan2(dy, dx);

    // Two-pass solve (refines muzzle offset)
    Translation3d muzzleStart = computeMuzzleStartField(robotPose, yawFieldGuess);
    BallisticAimSolver.Solution sol = BallisticAimSolver.solve(muzzleStart, hub, vRobotField);

    muzzleStart = computeMuzzleStartField(robotPose, sol.yawFieldRad);
    sol = BallisticAimSolver.solve(muzzleStart, hub, vRobotField);

    // Command turret yaw (robot-relative degrees)
    double turretYawRobotDeg = sol.turretYawRobotDeg(robotPose.getRotation().getRadians());
    turret.goToAngleDeg(turretYawRobotDeg);

    // Publish pitch (hood) to NT, and set shooter target RPM (SIM intent)
    hoodSim.setPitchDeg(sol.pitchDeg());
    shooter.setTargetRpm(sol.wheelRpm);

    // Fire exactly once per scheduling (do NOT wait for turret convergence in sim)
    if (!fired) {
      FuelSim.getInstance().spawnFuel(muzzleStart, sol.launchVelocityField);
      fired = true;
      fireTimeSec = Timer.getFPGATimestamp();

      System.out.println(
          "[SIM HUB SHOT] yawRobotDeg=" + turretYawRobotDeg
              + " pitchDeg=" + sol.pitchDeg()
              + " muzzleSpeed=" + sol.muzzleSpeedMps
              + " wheelRpm=" + sol.wheelRpm
              + " vLaunch=" + sol.launchVelocityField
              + " start=" + muzzleStart);
    }
  }

  @Override
  public boolean isFinished() {
    if (!fired)
      return false;
    return (Timer.getFPGATimestamp() - fireTimeSec) >= holdAfterFireSec;
  }

  @Override
  public void end(boolean interrupted) {
    // Intentionally do nothing: leaving turret/shooter commanded during hold window
    // is desired
  }

  private static Translation3d computeMuzzleStartField(Pose2d robotPose, double yawFieldRad) {
    // Turret origin offset in robot frame
    double tx = Constants.SimConstants.TURRET_X_M;
    double ty = Constants.SimConstants.TURRET_Y_M;
    double tz = Constants.SimConstants.TURRET_Z_M;

    // Muzzle offset in turret frame (rotated by yawField)
    double mx = Constants.SimConstants.MUZZLE_X_M;
    double my = Constants.SimConstants.MUZZLE_Y_M;
    double mz = Constants.SimConstants.MUZZLE_Z_M;

    // Rotate turret offset by robot yaw into field frame
    Rotation2d robotYaw = robotPose.getRotation();
    double cosR = robotYaw.getCos();
    double sinR = robotYaw.getSin();

    double turretFieldX = tx * cosR - ty * sinR;
    double turretFieldY = tx * sinR + ty * cosR;

    // Rotate muzzle offset by yawField into field frame
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
