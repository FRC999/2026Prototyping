package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;



public class TurretSubsystem extends SubsystemBase {
  private final TalonSRX turret = new TalonSRX(Constants.Turret.TALON_SRX_CAN_ID);

  // Field target tracking (Version-1)
  private volatile double targetFieldX = 0.0;
  private volatile double targetFieldY = 0.0;
  private volatile boolean hasFieldTarget = false;

  private double lastVelDegPerSec = 0.0;
  private double accelDegPerSec2 = 0.0;

  private final boolean isSim = RobotBase.isSimulation();

  private final FlywheelSim turretSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getVex775Pro(1),
          Constants.Turret.SIM_GEAR_RATIO,
          Constants.Turret.SIM_TURRET_J_KGM2
      ),
      DCMotor.getVex775Pro(1)
  );

private double simPosRad = 0.0;


  public TurretSubsystem() {
    configureTalon();

    // Seed relative encoder from absolute at startup.
    // Assumption per you: at boot, turret is within < 180° of forward.
    seedRelativeFromAbsolute();
  }

  private void configureTalon() {
    turret.configFactoryDefault();

    turret.setInverted(Constants.Turret.MOTOR_INVERTED);
    turret.setSensorPhase(Constants.Turret.SENSOR_PHASE);

    turret.setNeutralMode(NeutralMode.Brake);

    // Feedback sensor for PID: CTRE Mag Encoder Relative (quadrature)
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);

    // Output clamp
    turret.configPeakOutputForward(+Constants.Turret.MAX_OUTPUT, 50);
    turret.configPeakOutputReverse(-Constants.Turret.MAX_OUTPUT, 50);
    turret.configNominalOutputForward(0.0, 50);
    turret.configNominalOutputReverse(0.0, 50);

    // PIDF
    turret.config_kP(0, Constants.Turret.kP, 50);
    turret.config_kI(0, Constants.Turret.kI, 50);
    turret.config_kD(0, Constants.Turret.kD, 50);
    turret.config_kF(0, Constants.Turret.kF, 50);

    // Motion Magic (optional)
    turret.configMotionCruiseVelocity(Constants.Turret.MM_CRUISE_VEL, 50);
    turret.configMotionAcceleration(Constants.Turret.MM_ACCEL, 50);

    // Improve PID loop timing / status (optional)
    turret.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 50);
    turret.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, 50);

    // Software-side soft limits:
    // Talon SRX has forward/reverse soft limits in sensor units.
    // We'll configure them in RELATIVE ticks based on MIN/MAX degrees.
    int minTicks = degToTicks(Constants.Turret.MIN_ANGLE_DEG);
    int maxTicks = degToTicks(Constants.Turret.MAX_ANGLE_DEG);
    turret.configForwardSoftLimitThreshold(maxTicks, 50);
    turret.configReverseSoftLimitThreshold(minTicks, 50);
    turret.configForwardSoftLimitEnable(true, 50);
    turret.configReverseSoftLimitEnable(true, 50);

    turret.enableCurrentLimit(true);
    turret.configContinuousCurrentLimit(Constants.Turret.CONTINUOUS_CURRENT_LIMIT_A, 50);
    turret.configPeakCurrentLimit(Constants.Turret.PEAK_CURRENT_LIMIT_A, 50);
    turret.configPeakCurrentDuration(Constants.Turret.PEAK_CURRENT_DURATION_MS, 50);

  }

  /** Read absolute encoder ticks (0..4095) from PulseWidth. */
  public int getAbsoluteTicks() {
    // getPulseWidthPosition returns a value that typically wraps; mask to 12-bit range if needed.
    int raw = turret.getSensorCollection().getPulseWidthPosition();
    int abs = raw & 0xFFF; // 4095 mask
    return abs;
  }

  /** Relative position ticks (selected sensor). */
  public int getRelativeTicks() {
    return (int) Math.round(turret.getSelectedSensorPosition(0));
  }

  public double getAngleDeg() {
    return ticksToDeg(getRelativeTicks());
  }

  public void stop() {
    turret.set(ControlMode.PercentOutput, 0.0);
  }

  public void setOpenLoopPercent(double percent) {
    // clamp
    double p = Math.max(-Constants.Turret.MAX_OUTPUT, Math.min(Constants.Turret.MAX_OUTPUT, percent));

    // additional safety: prevent pushing further beyond limits (software clamp)
    if (wouldDriveFurtherOutOfBounds(p)) {
      stop();
      return;
    }
    turret.set(ControlMode.PercentOutput, p);
  }

  private boolean wouldDriveFurtherOutOfBounds(double percentOutput) {
    double angle = getAngleDeg();
    if (percentOutput > 0 && angle >= Constants.Turret.MAX_ANGLE_DEG) return true;
    if (percentOutput < 0 && angle <= Constants.Turret.MIN_ANGLE_DEG) return true;
    return false;
  }

  /** Hardware PID position control to a turret-relative angle (deg), within configured limits. */
  public void setTargetAngleDegPID(double targetDeg) {
    double safeDeg = clamp(targetDeg, Constants.Turret.MIN_ANGLE_DEG, Constants.Turret.MAX_ANGLE_DEG);
    turret.set(ControlMode.Position, degToTicks(safeDeg));
  }

  /** Motion Magic to angle (deg). */
  public void setTargetAngleDegMotionMagic(double targetDeg) {
    double safeDeg = clamp(targetDeg, Constants.Turret.MIN_ANGLE_DEG, Constants.Turret.MAX_ANGLE_DEG);
    turret.set(ControlMode.MotionMagic, degToTicks(safeDeg));
  }

  /**
   * "Smart" aiming: chooses an equivalent target angle by adding/subtracting 360°
   * so we don't cross a soft boundary unnecessarily.
   *
   * This assumes your allowed window is <= 360° wide (true for your ~380° physical but
   * you will set a safe window like ~340-380). If you allow >360 in software, rethink this.
   */
  public void setTargetAngleDegBestPath(double targetDeg, boolean useMotionMagic) {
    double current = getAngleDeg();

    double chosen = chooseEquivalentWithinLimits(current, targetDeg);
    if (useMotionMagic) setTargetAngleDegMotionMagic(chosen);
    else setTargetAngleDegPID(chosen);

    SmartDashboard.putNumber("Turret/ChosenTargetDeg", chosen);
  }

  private double chooseEquivalentWithinLimits(double currentDeg, double targetDeg) {
    // Try target + k*360 for k in {-1,0,1}
    double[] candidates = new double[] { targetDeg - 360.0, targetDeg, targetDeg + 360.0 };

    double best = Double.NaN;
    double bestDelta = Double.POSITIVE_INFINITY;

    for (double c : candidates) {
      if (c < Constants.Turret.MIN_ANGLE_DEG || c > Constants.Turret.MAX_ANGLE_DEG) continue;
      double d = Math.abs(c - currentDeg);
      if (d < bestDelta) {
        bestDelta = d;
        best = c;
      }
    }

    // If none fit, clamp (this means your request is outside the safe window)
    if (Double.isNaN(best)) {
      best = clamp(targetDeg, Constants.Turret.MIN_ANGLE_DEG, Constants.Turret.MAX_ANGLE_DEG);
    }
    return best;
  }

  /** Re-seed the RELATIVE encoder so that "forward" absolute = 0 relative ticks. */
  public void seedRelativeFromAbsolute() {
    int abs = getAbsoluteTicks();
    int forward = Constants.Turret.ABS_FORWARD_TICKS;

    // compute shortest signed delta from forward to current, in absolute ticks (-2048..+2047)
    int delta = shortestSignedAbsDeltaTicks(forward, abs);

    // convert delta absolute ticks to degrees using your placeholder ticks/deg.
    // Here is a subtlety: absolute ticks are 4096/rev, but your REL ticks/deg is separate.
    // We only need an internal consistent relative zero reference; we map the boot delta into REL ticks.
    double deltaDeg = (delta / (double) Constants.Turret.ABS_TICKS_PER_REV) * 360.0;
    int relTicks = degToTicks(deltaDeg);

    // Set selected sensor position so that forward=0 => current is delta from forward
    turret.setSelectedSensorPosition(relTicks, 0, 50);

    SmartDashboard.putNumber("Turret/SeedAbsTicks", abs);
    SmartDashboard.putNumber("Turret/SeedDeltaDeg", deltaDeg);
    SmartDashboard.putNumber("Turret/SeedRelTicks", relTicks);
  }

  private int shortestSignedAbsDeltaTicks(int fromTicks, int toTicks) {
    int diff = toTicks - fromTicks;
    diff = ((diff + (Constants.Turret.ABS_TICKS_PER_REV / 2)) % Constants.Turret.ABS_TICKS_PER_REV)
        - (Constants.Turret.ABS_TICKS_PER_REV / 2);
    return diff;
  }

  public boolean atSetpoint(double targetDeg) {
    return Math.abs(getAngleDeg() - targetDeg) <= Constants.Turret.AT_SETPOINT_DEG;
  }

  // ---------------- Field target math ----------------

  public void setFieldTarget(double tx, double ty) {
    targetFieldX = tx;
    targetFieldY = ty;
    hasFieldTarget = true;
  }

  public void clearFieldTarget() {
    hasFieldTarget = false;
  }

  public boolean hasFieldTarget() {
    return hasFieldTarget;
  }

  /**
   * Compute required turret-relative angle (deg) so that the turret (located at DX,DY in robot frame)
   * points at the field point (tx,ty).
   *
   * Returns angle in degrees, where 0 means "forward" in robot frame, CCW positive.
   */
  public double computeTurretAngleToFieldPointDeg(Pose2d robotPose, double tx, double ty) {
    Translation2d robotTranslation = robotPose.getTranslation();
    Rotation2d robotRot = robotPose.getRotation();

    // turret position in robot frame -> rotate into field and add
    Translation2d turretOffsetRobot = new Translation2d(Constants.Turret.DX_METERS, Constants.Turret.DY_METERS);
    Translation2d turretPosField = robotTranslation.plus(turretOffsetRobot.rotateBy(robotRot));

    Translation2d targetField = new Translation2d(tx, ty);
    Translation2d toTarget = targetField.minus(turretPosField);

    Rotation2d fieldAngleToTarget = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));

    // turret relative angle = field angle - robot heading
    Rotation2d turretRel = fieldAngleToTarget.minus(robotRot);
    return turretRel.getDegrees();
  }

  // ---------------- Utils ----------------

  private static int degToTicks(double deg) {
    return (int) Math.round(deg * Constants.Turret.REL_TICKS_PER_DEG);
  }

  private static double ticksToDeg(int ticks) {
    return ticks / Constants.Turret.REL_TICKS_PER_DEG;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  public double getVelocityDegPerSec() {
    // Talon SRX velocity is ticks per 100ms
    double ticksPer100ms = turret.getSelectedSensorVelocity(0);
    double ticksPerSec = ticksPer100ms * 10.0;
    return ticksPerSec / Constants.Turret.REL_TICKS_PER_DEG;
  }

  public double getAccelerationDegPerSec2() {
    return accelDegPerSec2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/AbsTicks", getAbsoluteTicks());
    SmartDashboard.putNumber("Turret/RelTicks", getRelativeTicks());
    SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
    SmartDashboard.putBoolean("Turret/HasFieldTarget", hasFieldTarget);

    double dt = 0.02;
    double vel = getVelocityDegPerSec();
    double rawAccel = (vel - lastVelDegPerSec) / dt;
    accelDegPerSec2 = accelDegPerSec2 + Constants.Turret.ACCEL_ALPHA * (rawAccel - accelDegPerSec2);
    lastVelDegPerSec = vel;

    SmartDashboard.putNumber("Turret/VelDegPerSec", vel);
    SmartDashboard.putNumber("Turret/AccelDegPerSec2", accelDegPerSec2);

  }

  @Override
  public void simulationPeriodic() {
    // Talon SRX sim collection
    var sim = turret.getSimCollection();

    // Motor input voltage approximation: battery * applied output
    double batteryV = RoboRioSim.getVInVoltage();
    double applied = sim.getMotorOutputLeadVoltage(); // in sim, this reflects output; if 0, fallback:
    if (Math.abs(applied) < 1e-6) {
      applied = turret.getMotorOutputPercent() * batteryV;
    }

    turretSim.setInputVoltage(applied);
    turretSim.update(0.02);

    // Integrate position
    double omegaRadPerSec = turretSim.getAngularVelocityRadPerSec();
    simPosRad += omegaRadPerSec * 0.02;

    // Convert sim position to your REL ticks (deg->ticks). Keep consistent with your convention.
    double posDeg = Math.toDegrees(simPosRad);
    int relTicks = (int)Math.round(posDeg * Constants.Turret.REL_TICKS_PER_DEG);

    // Velocity in ticks/100ms (CTRE units)
    double velDegPerSec = Math.toDegrees(omegaRadPerSec);
    int velTicksPer100ms = (int)Math.round((velDegPerSec * Constants.Turret.REL_TICKS_PER_DEG) / 10.0);

    sim.setQuadratureRawPosition(relTicks);
    sim.setQuadratureVelocity(velTicksPer100ms);

    // Update battery under load
    double currentDraw = turretSim.getCurrentDrawAmps();
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currentDraw));
  }

}
