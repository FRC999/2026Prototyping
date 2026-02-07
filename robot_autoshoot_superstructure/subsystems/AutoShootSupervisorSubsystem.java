
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TurretHelpers;

/**
 * AutoShootSupervisorSubsystem
 *
 * "Superstructure" owner of the volley state machine.
 *
 * Why a subsystem owns state (instead of the command):
 * - Commands are interruptible; you still want the system to "know what it was doing" for telemetry/debug.
 * - The driver can press the shoot button multiple times; we want deterministic behavior.
 *
 * High-level goals:
 * - Aim turret at target at all times (if enabled).
 * - When the driver requests shooting, run a continuous volley:
 *   keep shooter + hood + turret commanded; stage balls; fire as soon as gates are satisfied; repeat until empty.
 * - If no shooting solution exists for the next ball, the volley pauses (NO_SOLUTION) instead of firing blind.
 *
 * Performance architecture:
 * - This subsystem does the math once per 20 ms loop (50 Hz). The solver itself is lightweight.
 * - Artillery table is loaded once at startup from /deploy (CSV).
 * - Robot acceleration is estimated from two consecutive velocity samples (no CTRE acceleration signal needed).
 */
public class AutoShootSupervisorSubsystem extends SubsystemBase {

  public enum VolleyState {
    IDLE,
    ARMING,        // preparing + staging concurrently
    FIRING,        // actively feeding ball into shooter
    RECOVERING,    // waiting for shooter to recover after a dip
    NO_SOLUTION,   // requested shoot, but no valid solution exists (pause)
    EMPTY          // no balls remaining
  }

  private final DriveSubsystem drive;
  private final TurretSubsystem turret;
  private final ShooterSubsystem shooter;
  private final SpindexerSubsystem spindexer;
  private final TransferSubsystem transfer;

  private final TurretHelpers.ArtilleryTableIndexedByShooterRpmAndHoodAngle table;

  // Driver request flag
  private boolean shootRequested = false;

  private VolleyState state = VolleyState.IDLE;

  // Ball estimate (until you add true sensors for hopper count)
  private int ballsRemaining = 0;
  private double lastDipTs = -1.0;

  // Estimated field acceleration
  private double lastVelXField = 0.0;
  private double lastVelYField = 0.0;
  private double lastVelTs = -1.0;

  // Soft-limit flip suppression
  private boolean avoidingEdge = false;
  private double suppressShootUntilTs = 0.0;

  // Cached desired turret command each loop (deg in turret-forward frame)
  private double desiredTurretDeg = Double.NaN;

  // Cached solver output
  private TurretHelpers.Solution lastSolution = TurretHelpers.makeInvalidSolution();

  public AutoShootSupervisorSubsystem(
      DriveSubsystem drive,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      SpindexerSubsystem spindexer,
      TransferSubsystem transfer
  ) {
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.spindexer = spindexer;
    this.transfer = transfer;

    // Load artillery table once. If missing/empty, hasAnyData() will be false and solver will return invalid.
    this.table = TurretHelpers.ArtilleryTableIndexedByShooterRpmAndHoodAngle
        .loadFromDeployCsv(Constants.OperatorConstants.ArtilleryTable.DEPLOY_CSV_PATH);
  }

  /** Driver intent: true = attempt to run a volley; false = stop shooting immediately. */
  public void setShootRequested(boolean requested) {
    if (requested && !shootRequested) {
      // Rising edge: latch ball estimate at start of volley.
      latchBallsRemainingFromDashboard();
      avoidingEdge = false;
      suppressShootUntilTs = 0.0;
      lastDipTs = -1.0;
    }
    shootRequested = requested;
    if (!shootRequested) {
      // Drop everything safely; keep aiming if ALWAYS_AIM is enabled.
      transfer.stop();
      spindexer.stop();
      shooter.stopFeederRelatedOutputs(); // placeholder method (no-op if you don't need it)
    }
  }

  public boolean isShootRequested() {
    return shootRequested;
  }

  public VolleyState getVolleyState() {
    return state;
  }

  public TurretHelpers.Solution getLastSolution() {
    return lastSolution;
  }

  private void latchBallsRemainingFromDashboard() {
    int fromDash = (int) SmartDashboard.getNumber(
        "Hopper/BallsEstimate",
        Constants.OperatorConstants.AutoShoot.DEFAULT_BALLS_ESTIMATE);
    ballsRemaining = Math.max(0, fromDash);
    SmartDashboard.putNumber("Hopper/BallsEstimate", ballsRemaining);
  }

  @Override
  public void periodic() {
    final double now = Timer.getFPGATimestamp();

    // --- 1) Compute target position ---
    Translation2d target2d = getAllianceHubTarget();
    Translation3d target3d = new Translation3d(
        target2d.getX(),
        target2d.getY(),
        Constants.OperatorConstants.FieldGeometry.HUB_OPENING_CENTER_Z_METERS);

    // --- 2) Read drive state and estimate field velocity + acceleration ---
    var driveState = drive.getState();
    var poseField = driveState.Pose;

    // CTRE state.Speeds is robot-relative chassis speeds; convert to FIELD frame.
    double vxRobot = driveState.Speeds.vxMetersPerSecond;
    double vyRobot = driveState.Speeds.vyMetersPerSecond;
    Translation2d vField = new Translation2d(vxRobot, vyRobot).rotateBy(poseField.getRotation());
    double omega = driveState.Speeds.omegaRadiansPerSecond;

    Translation2d aField = estimateAccelerationField(now, vField);

    // --- 3) Solve shooting (or just aim) ---
    lastSolution = TurretHelpers.solveForShooterRpmAndHoodAngleCommandsWhileRobotIsMovingUsingMeasuredTableIndexedByRpmAndHood(
        poseField,
        vField,
        aField,
        omega,
        Constants.OperatorConstants.AutoShoot.DT_RELEASE_SEC,
        Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS,
        Constants.OperatorConstants.TurretGeometry.BALL_RELEASE_HEIGHT_METERS,
        target3d,
        table,
        Constants.OperatorConstants.ArtillerySolver.TOF_MIN_SEC,
        Constants.OperatorConstants.ArtillerySolver.TOF_MAX_SEC,
        Constants.OperatorConstants.ArtillerySolver.TOF_STEP_SEC,
        Constants.OperatorConstants.ArtillerySolver.GRAVITY_MPS2,
        Constants.OperatorConstants.ArtillerySolver.ANGLE_WEIGHT,
        Constants.OperatorConstants.ArtillerySolver.SPEED_WEIGHT
    );

    boolean solutionValid = lastSolution.valid;

    // Compute desired turret angle now (deg in turret-forward frame) using predicted robot heading at release time.
    desiredTurretDeg = computeDesiredTurretDeg(
        poseField.getRotation().getRadians(),
        omega,
        Constants.OperatorConstants.AutoShoot.DT_RELEASE_SEC,
        lastSolution.yawFieldRad,
        Constants.OperatorConstants.Turret.ZERO_POINTS_ROBOT_BACK
    );

    desiredTurretDeg = chooseSoftLimitedEquivalent(desiredTurretDeg, now);

    // Always aim if enabled OR if shooting is requested.
    boolean aimEnabled = Constants.OperatorConstants.AutoShoot.ALWAYS_AIM || shootRequested;
    if (aimEnabled && Double.isFinite(desiredTurretDeg)) {
      turret.goToAngleDeg(desiredTurretDeg);
    }

    // --- 4) Decide state machine ---
    updateBallCountFromShooterDip(now);

    boolean empty = ballsRemaining <= 0;
    boolean suppress = now < suppressShootUntilTs;

    boolean turretAimed = isTurretAimed(desiredTurretDeg);
    boolean shooterReady = shooter.isReadyToShoot();
    boolean ballAtThroat = transfer.hasBallAtThroat();

    SmartDashboard.putBoolean("AutoShoot/SolutionValid", solutionValid);
    SmartDashboard.putBoolean("AutoShoot/TurretAimed", turretAimed);
    SmartDashboard.putBoolean("AutoShoot/ShooterReady", shooterReady);
    SmartDashboard.putBoolean("AutoShoot/BallAtThroat", ballAtThroat);
    SmartDashboard.putBoolean("AutoShoot/Suppress", suppress);

    // If not requested, keep system safe.
    if (!shootRequested) {
      state = VolleyState.IDLE;
      // Keep stage gentle only if you explicitly want pre-staging without a shoot request.
      transfer.stop();
      spindexer.stop();
      // Shooter can remain off when not requested.
      shooter.stop();
      publishTelemetry();
      return;
    }

    if (empty) {
      state = VolleyState.EMPTY;
      transfer.stop();
      spindexer.stop();
      shooter.stop();
      publishTelemetry();
      return;
    }

    if (!solutionValid) {
      state = VolleyState.NO_SOLUTION;
      // Keep staging and aiming but do not feed into shooter.
      spindexer.runBase();
      transfer.runStage();
      shooter.stop(); // don't spin blindly if you don't have a solution
      publishTelemetry();
      return;
    }

    // We have a solution: command shooter + hood.
    shooter.setTargetRpm(lastSolution.shooterRpm);
    turret.setHoodAngleRad(lastSolution.hoodAngleRad); // placeholder if hood lives elsewhere

    // Concurrent staging: keep a ball at throat as much as possible.
    spindexer.runSupply();

    // If suppressing due to flip/unwrap, do NOT fire.
    if (suppress) {
      state = VolleyState.ARMING;
      transfer.runStage();
      publishTelemetry();
      return;
    }

    // Gate to actually fire:
    boolean okToFire = turretAimed && shooterReady && ballAtThroat;

    if (okToFire) {
      state = VolleyState.FIRING;
      transfer.runFeed();
    } else {
      state = VolleyState.ARMING;
      // Keep staged; if ball not at throat yet, keep moving it.
      transfer.runStage();
    }

    // If we just detected a dip, move to recovering (prevents double-feeding).
    if (state == VolleyState.FIRING && shooter.wasDipDetected()) {
      state = VolleyState.RECOVERING;
      transfer.stop(); // stop feeding while RPM recovers
      shooter.clearDipDetected();
      lastDipTs = now;
    }

    // Recovering: wait until shooter ready again, then resume staging/firing.
    if (state == VolleyState.RECOVERING) {
      if (shooterReady) {
        state = VolleyState.ARMING;
      } else {
        transfer.stop();
      }
    }

    publishTelemetry();
  }

  private Translation2d estimateAccelerationField(double now, Translation2d vField) {
    if (lastVelTs < 0) {
      lastVelTs = now;
      lastVelXField = vField.getX();
      lastVelYField = vField.getY();
      return new Translation2d(0.0, 0.0);
    }
    double dt = Math.max(1e-3, now - lastVelTs);
    double ax = (vField.getX() - lastVelXField) / dt;
    double ay = (vField.getY() - lastVelYField) / dt;
    lastVelTs = now;
    lastVelXField = vField.getX();
    lastVelYField = vField.getY();
    return new Translation2d(ax, ay);
  }

  private void updateBallCountFromShooterDip(double now) {
    if (!shootRequested) return;

    if (shooter.wasDipDetected()) {
      if (lastDipTs < 0 || (now - lastDipTs) >= Constants.OperatorConstants.AutoShoot.DIP_DEBOUNCE_S) {
        ballsRemaining = Math.max(0, ballsRemaining - 1);
        SmartDashboard.putNumber("Hopper/BallsEstimate", ballsRemaining);
        lastDipTs = now;
      }
      // Do not clear here; state machine may clear depending on firing behavior.
      // But clear anyway so we don't double-count.
      shooter.clearDipDetected();
    }
  }

  private static Translation2d getAllianceHubTarget() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    if (isRed) {
      return new Translation2d(Constants.FieldTargets.HUB_RED_X, Constants.FieldTargets.HUB_RED_Y);
    }
    return new Translation2d(Constants.FieldTargets.HUB_BLUE_X, Constants.FieldTargets.HUB_BLUE_Y);
  }

  /**
   * Convert a desired field yaw (radians) into turret-forward-frame degrees.
   *
   * Steps:
   * - predict robot heading at release time: headingNow + omega * dtRelease
   * - compute field->robot relative angle: yawField - predictedHeading
   * - shift by 180 deg if turret zero points robot BACK
   * - output degrees in (-180..+180] then allow caller to choose equivalent ±360
   */
  private static double computeDesiredTurretDeg(
      double robotHeadingFieldRad,
      double omegaRadPerSec,
      double dtReleaseSec,
      double desiredYawFieldRad,
      boolean zeroPointsRobotBack
  ) {
    double predictedHeading = robotHeadingFieldRad + omegaRadPerSec * dtReleaseSec;
    double robotRelative = MathUtil.angleModulus(desiredYawFieldRad - predictedHeading);
    if (zeroPointsRobotBack) {
      robotRelative = MathUtil.angleModulus(robotRelative - Math.PI);
    }
    return Math.toDegrees(robotRelative);
  }

  /** Simple aim check: compare current continuous turret angle to desired. */
  private boolean isTurretAimed(double desiredDeg) {
    if (!Double.isFinite(desiredDeg)) return false;
    double err = Math.abs(desiredDeg - turret.getContinuousAngleDeg());
    return err <= Constants.OperatorConstants.Turret.AIM_TOLERANCE_DEG;
  }

  /**
   * Soft-limit selection to avoid living at the ends of turret travel.
   *
   * Also implements "flip suppression": if we unwrap/flip, we suppress feeding briefly so we don't fire mid-swing.
   */
  private double chooseSoftLimitedEquivalent(double desiredDeg, double nowTs) {
    double soft = Constants.OperatorConstants.Turret.SOFT_AIM_LIMIT_DEG;
    double margin = Constants.OperatorConstants.Turret.LIMIT_MARGIN_DEG;

    double currentDeg = turret.getContinuousAngleDeg();

    double[] cands = new double[] { desiredDeg, desiredDeg + 360.0, desiredDeg - 360.0 };
    double best = Double.NaN;
    double bestScore = Double.POSITIVE_INFINITY;

    for (double c : cands) {
      if (c < -soft || c > soft) continue;
      double travel = Math.abs(c - currentDeg);

      // Edge penalty: discourage living within 'margin' of the soft ends.
      double edgeDist = Math.min(Math.abs(soft - c), Math.abs(-soft - c));
      double edgePenalty = edgeDist < margin ? (margin - edgeDist) * 5.0 : 0.0;

      double score = travel + edgePenalty;
      if (score < bestScore) {
        bestScore = score;
        best = c;
      }
    }

    if (Double.isNaN(best)) {
      // If the target can't be reached within the soft range, clamp.
      best = MathUtil.clamp(desiredDeg, -soft, soft);
    }

    // Flip state + shoot suppression when near edge.
    boolean nearEdgeNow = Math.abs(best) >= (soft - margin);
    if (nearEdgeNow && !avoidingEdge) {
      avoidingEdge = true;
      suppressShootUntilTs = nowTs + Constants.OperatorConstants.AutoShoot.FLIP_SUPPRESS_SEC;
    }
    if (!nearEdgeNow && avoidingEdge) {
      avoidingEdge = false;
    }

    // If we chose a different branch (±360), it will look like a big step; suppress briefly.
    if (Math.abs(best - desiredDeg) > 180.0) {
      suppressShootUntilTs = Math.max(
          suppressShootUntilTs,
          nowTs + Constants.OperatorConstants.AutoShoot.DT_RELEASE_SEC);
    }

    return best;
  }

  private void publishTelemetry() {
    SmartDashboard.putString("AutoShoot/State", state.toString());
    SmartDashboard.putNumber("AutoShoot/BallsRemaining", ballsRemaining);
    SmartDashboard.putNumber("AutoShoot/DesiredTurretDeg", desiredTurretDeg);
    SmartDashboard.putNumber("AutoShoot/SuppressUntilTs", suppressShootUntilTs);
    SmartDashboard.putNumber("AutoShoot/LastSol/Rpm", lastSolution.shooterRpm);
    SmartDashboard.putNumber("AutoShoot/LastSol/HoodDeg", Math.toDegrees(lastSolution.hoodAngleRad));
    SmartDashboard.putNumber("AutoShoot/LastSol/YawFieldDeg", Math.toDegrees(lastSolution.yawFieldRad));
  }
}
