package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class BallisticAimSolver {

  /** Output of the solver. All angles are field-relative unless specified. */
  public static class Solution {
    public final double timeOfFlightS;

    // Field-relative yaw (radians) and pitch (radians)
    public final double yawFieldRad;
    public final double pitchRad;

    // Speed of the ball relative to the robot (m/s)
    public final double muzzleSpeedMps;

    // Ball initial velocity in the FIELD frame (m/s) including robot motion
    public final Translation3d launchVelocityField;

    // Optional: what wheel RPM would roughly produce this muzzle speed
    public final double wheelRpm;

    public Solution(
        double timeOfFlightS,
        double yawFieldRad,
        double pitchRad,
        double muzzleSpeedMps,
        Translation3d launchVelocityField,
        double wheelRpm) {
      this.timeOfFlightS = timeOfFlightS;
      this.yawFieldRad = yawFieldRad;
      this.pitchRad = pitchRad;
      this.muzzleSpeedMps = muzzleSpeedMps;
      this.launchVelocityField = launchVelocityField;
      this.wheelRpm = wheelRpm;
    }

    /** Robot-relative turret yaw in degrees (0 = robot forward, CCW positive). */
    public double turretYawRobotDeg(double robotYawRad) {
      double rel = yawFieldRad - robotYawRad;
      // wrap to [-pi, pi] for a "shortest" command, turret subsystem will handle limits
      rel = Math.atan2(Math.sin(rel), Math.cos(rel));
      return Math.toDegrees(rel);
    }

    public double pitchDeg() {
      return Math.toDegrees(pitchRad);
    }
  }

  /**
   * Solve for a ballistic shot from p0 to pt given robot field velocity vRobotField.
   *
   * Model:
   *   pt = p0 + (vRobot + u)*T + 0.5*g*T^2
   * where:
   *   u = muzzle-relative velocity (what shooter imparts), in FIELD coordinates
   *   g = (0,0,-9.81)
   *
   * For each time-of-flight T, we compute u:
   *   u = (pt - p0 - vRobot*T - 0.5*g*T^2)/T
   *
   * Then we compute yaw/pitch/speed from u, and keep the best feasible solution.
   */
  public static Solution solve(
      Translation3d p0Field,
      Translation3d ptField,
      Translation3d vRobotField) {

    final double g = 9.81;
    final double pitchMin = Math.toRadians(Constants.SimConstants.PITCH_MIN_DEG);
    final double pitchMax = Math.toRadians(Constants.SimConstants.PITCH_MAX_DEG);

    Solution best = null;
    double bestSpeed = Double.POSITIVE_INFINITY;

    for (double T = Constants.SimConstants.TOF_MIN_S;
         T <= Constants.SimConstants.TOF_MAX_S;
         T += Constants.SimConstants.TOF_STEP_S) {

      // delta = target - start
      double dx = ptField.getX() - p0Field.getX();
      double dy = ptField.getY() - p0Field.getY();
      double dz = ptField.getZ() - p0Field.getZ();

      // u = (delta - vRobot*T - 0.5*g*T^2)/T
      double ux = (dx - vRobotField.getX() * T) / T;
      double uy = (dy - vRobotField.getY() * T) / T;
      double uz = (dz - vRobotField.getZ() * T + 0.5 * g * T * T) / T; // + because g is downward

      // Solve yaw/pitch from u
      double yaw = Math.atan2(uy, ux);
      double horiz = Math.hypot(ux, uy);
      double pitch = Math.atan2(uz, horiz);
      double speed = Math.sqrt(ux*ux + uy*uy + uz*uz);

      // Enforce pitch constraints
      if (pitch < pitchMin || pitch > pitchMax) continue;

      // Enforce max speed feasibility
      if (speed > Constants.SimConstants.MUZZLE_SPEED_MAX_MPS) continue;

      // Keep smallest speed (more "reasonable" shot)
      if (speed < bestSpeed) {
        bestSpeed = speed;

        // total launch velocity = vRobot + u (field frame)
        Translation3d vLaunch =
            new Translation3d(
                vRobotField.getX() + ux,
                vRobotField.getY() + uy,
                vRobotField.getZ() + uz + Constants.SimConstants.SHOT_EXTRA_UP_V_MPS);

        double wheelRpm = muzzleSpeedToWheelRpm(speed);

        best = new Solution(T, yaw, pitch, speed, vLaunch, wheelRpm);
      }
    }

    // If no feasible solution found, still produce something:
    // pick yaw directly at target and clamp speed so it "tries" to go in.
    if (best == null) {
      double dx = ptField.getX() - p0Field.getX();
      double dy = ptField.getY() - p0Field.getY();
      double dz = ptField.getZ() - p0Field.getZ();

      double yaw = Math.atan2(dy, dx);

      // choose mid pitch and solve for speed using that pitch & a reasonable TOF
      double pitch = Math.toRadians(
          (Constants.SimConstants.PITCH_MIN_DEG + Constants.SimConstants.PITCH_MAX_DEG) * 0.5);

      double T = 1.0;

      // Given yaw & pitch, solve speed from XY displacement ignoring drag:
      // horizDist = speed*cos(pitch)*T, but T is unknown; we keep T=1 to avoid crazy math here.
      // This fallback is just to avoid "no shot".
      double horizDist = Math.hypot(dx, dy);
      double speed = Math.min(Constants.SimConstants.MUZZLE_SPEED_CLAMP_MPS, horizDist / (Math.cos(pitch) * T + 1e-6));

      double ux = Math.cos(yaw) * Math.cos(pitch) * speed;
      double uy = Math.sin(yaw) * Math.cos(pitch) * speed;

      // Choose uz so that z is closer (includes gravity)
      double uz = (dz + 0.5 * g * T * T) / T;

      Translation3d vLaunch =
          new Translation3d(
              vRobotField.getX() + ux,
              vRobotField.getY() + uy,
              vRobotField.getZ() + uz);

      double wheelRpm = muzzleSpeedToWheelRpm(speed);

      best = new Solution(T, yaw, pitch, speed, vLaunch, wheelRpm);
    }

    return best;
  }

  private static double muzzleSpeedToWheelRpm(double muzzleSpeedMps) {
    // muzzleSpeed ≈ surfaceSpeed * efficiency
    // surfaceSpeed = 2πR * (rpm/60)
    double eff = Constants.SimConstants.SHOOTER_TO_BALL_EFFICIENCY;
    double R = Constants.SimConstants.SHOOTER_WHEEL_RADIUS_M;

    if (R <= 1e-6 || eff <= 1e-6) return 0.0;

    double surfaceSpeed = muzzleSpeedMps / eff;
    double rps = surfaceSpeed / (2.0 * Math.PI * R);
    return rps * 60.0;
  }
}
