package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants;

/** Turret prototype (Talon FXS + CTRE Mag encoder). */
public class TurretSubsystem extends SubsystemBase {

  private final TalonFXS turret = new TalonFXS(
      Constants.OperatorConstants.Turret.CAN_ID,
      Constants.OperatorConstants.Turret.CANBUS_NAME);

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  // Typed cached signals (Phoenix 6 returns typed signals)
  private final StatusSignal<Angle> positionSig = turret.getPosition();
  private final StatusSignal<AngularVelocity> velocitySig = turret.getVelocity();
  private final StatusSignal<Voltage> motorVoltageSig = turret.getMotorVoltage();

  // This one is the troublemaker if you refresh() it every 20ms.
  private final StatusSignal<Angle> rawPwmPosSig = turret.getRawPulseWidthPosition();

  private boolean useMotionMagic = true;

  private final boolean isSim = RobotBase.isSimulation();

  // Simulation position accumulator (rotations, continuous)
  private double simPosRot = 0.0;

  // Cache absolute ticks so we don't hard-refresh PWM continuously
  private int lastAbsTicks = 0;
  private double lastAbsRefreshTime = -1.0;

  // How often to refresh PWM absolute (seconds). 0.2s = 5 Hz.
  private static final double ABS_REFRESH_PERIOD_S = 0.20;

  // Modern WPILib FlywheelSim constructor (plant + motor model)
  private final FlywheelSim turretSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getVex775Pro(1),
          Constants.OperatorConstants.Turret.SIM_GEAR_RATIO,
          Constants.OperatorConstants.Turret.SIM_TURRET_J_KGM2),
      DCMotor.getVex775Pro(1));

  // ---------------- SysId Characterization ----------------
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Seconds).of(Constants.OperatorConstants.SysId.TURRET_RAMP_RATE_V_PER_S),
          Volts.of(Constants.OperatorConstants.SysId.TURRET_STEP_V),
          Seconds.of(Constants.OperatorConstants.SysId.TURRET_TIMEOUT_S)),
      new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "turret"));

  /**
   * Runtime gating for SysId. Requires BOTH compile-time enable and dashboard
   * enable.
   * This prevents accidental characterization runs.
   */
  private boolean isSysIdEnabled() {
    return Constants.OperatorConstants.SysId.ENABLE_SYSID
        && SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public TurretSubsystem() {
    configureHardware();

    // Configure signal update rates BEFORE we start calling refresh() in periodic.
    configureStatusSignals();

    // Seed relative from absolute ONCE at boot.
    seedRelativeFromAbsoluteAtBoot();

    // After boot seeding, we still keep PWM visible, but only refreshed slowly.
    // (The slow refresh is handled by periodic()).
  }

  private void configureStatusSignals() {
    // Fast signals (closed-loop control & dashboard)
    positionSig.setUpdateFrequency(100.0); // Hz
    velocitySig.setUpdateFrequency(100.0);
    motorVoltageSig.setUpdateFrequency(50.0);

    // Raw PWM absolute is only needed for boot seeding + occasional debugging.
    rawPwmPosSig.setUpdateFrequency(10.0);

    // Reduce CAN spam by only sending what we asked for.
    turret.optimizeBusUtilization();
  }

  private void configureHardware() {
    MotorOutputConfigs out = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(
            Constants.OperatorConstants.Turret.MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LIMIT_A)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.OperatorConstants.Turret.STATOR_CURRENT_LIMIT_A);

    // Closed-loop: ensure continuous wrap is OFF (we enforce umbilical safe limits)
    ClosedLoopGeneralConfigs clGen = new ClosedLoopGeneralConfigs().withContinuousWrap(false);

    MotionMagicConfigs mm = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(Constants.OperatorConstants.Turret.MM_CRUISE_VEL_RPS)
        .withMotionMagicAcceleration(Constants.OperatorConstants.Turret.MM_ACCEL_RPS2);

    // Use quadrature for control (multi-turn). CTRE Mag is 4096 edges per rotation.
    ExternalFeedbackConfigs ext = new ExternalFeedbackConfigs()
        .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Quadrature)
        .withQuadratureEdgesPerRotation(Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV)
        .withSensorPhase(SensorPhaseValue.Aligned);

    Slot0Configs slot0 = new Slot0Configs()
        .withKP(Constants.OperatorConstants.Turret.kP)
        .withKI(Constants.OperatorConstants.Turret.kI)
        .withKD(Constants.OperatorConstants.Turret.kD)
        .withKS(Constants.OperatorConstants.Turret.kS)
        .withKV(Constants.OperatorConstants.Turret.kV)
        .withKA(Constants.OperatorConstants.Turret.kA);

    // Software limits in rotations (relative, seeded so 0 = forward).
    double minRot = Constants.OperatorConstants.Turret.MIN_ANGLE_DEG / 360.0;
    double maxRot = Constants.OperatorConstants.Turret.MAX_ANGLE_DEG / 360.0;
    SoftwareLimitSwitchConfigs soft = new SoftwareLimitSwitchConfigs()
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(minRot)
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxRot);

    CommutationConfigs commutation = new CommutationConfigs()
        .withMotorArrangement(MotorArrangementValue.Brushed_DC)
        .withBrushedMotorWiring(BrushedMotorWiringValue.Leads_A_and_B);


    TalonFXSConfiguration cfg = new TalonFXSConfiguration()
        .withMotorOutput(out)
        .withCurrentLimits(limits)
        .withClosedLoopGeneral(clGen)
        .withExternalFeedback(ext)
        .withSoftwareLimitSwitch(soft)
        .withMotionMagic(mm)
        .withSlot0(slot0)
        .withCommutation(commutation);

    turret.getConfigurator().apply(cfg);
  }

  /**
   * Seeds the multi-turn (quadrature) relative position so that 0 degrees means
   * "forward".
   * Assumes at boot the turret is within +/- 180 degrees of forward.
   */
private void seedRelativeFromAbsoluteAtBoot() {
  // Temporarily switch to PulseWidth to read absolute.
  ExternalFeedbackConfigs pwm = new ExternalFeedbackConfigs()
      .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.PulseWidth)
      .withAbsoluteSensorDiscontinuityPoint(1.0)
      .withSensorPhase(SensorPhaseValue.Aligned);

  //Apply ONLY this config (do NOT wrap in a new TalonFXSConfiguration)
  turret.getConfigurator().apply(pwm);

  int absTicks = readAbsoluteTicksFresh(0.30);
  lastAbsTicks = absTicks;
  lastAbsRefreshTime = Timer.getFPGATimestamp();

  int deltaTicks = wrapToHalfRev(
      absTicks - Constants.OperatorConstants.Turret.ABS_FORWARD_TICKS,
      Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV);
  double deltaRot = (double) deltaTicks / (double) Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;

  // Restore quadrature config and set relative position
  ExternalFeedbackConfigs quad = new ExternalFeedbackConfigs()
      .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Quadrature)
      .withQuadratureEdgesPerRotation(Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV)
      .withSensorPhase(SensorPhaseValue.Aligned);

  // ✅ Apply ONLY this config (preserves commutation)
  turret.getConfigurator().apply(quad);

  turret.setPosition(deltaRot);
}

  /**
   * Attempts to read a fresh absolute PWM tick value, retrying briefly.
   * This is used only during boot seeding.
   */
  private int readAbsoluteTicksFresh(double timeoutSec) {
    double start = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() - start < timeoutSec) {
      rawPwmPosSig.refresh();
      StatusCode status = rawPwmPosSig.getStatus();
      if (status == StatusCode.OK) {

        return angleRotToAbsTicks(rawPwmPosSig.getValueAsDouble());
      }
      // small delay so we don't hammer CAN
      Timer.delay(0.01);
    }

    // If we couldn't get fresh data, fall back to the last cached ticks (0 on first
    // boot)
    return lastAbsTicks;
  }

  /** Converts a rotations value (0..1..wrapped) to [0..4095] ticks. */
  private int angleRotToAbsTicks(double rot) {
    rot = rot - Math.floor(rot); // wrap into [0,1)
    int ticks = (int) Math.round(rot * Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV);
    ticks %= Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;
    if (ticks < 0)
      ticks += Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;
    return ticks;
  }

  /**
   * Refreshes PWM absolute only occasionally to avoid -1003 "too stale" spam.
   * Keeps lastAbsTicks updated at low rate, and never throws if stale.
   */
  private void updateAbsoluteTicksOccasionally() {
    double now = Timer.getFPGATimestamp();
    if (lastAbsRefreshTime >= 0 && (now - lastAbsRefreshTime) < ABS_REFRESH_PERIOD_S)
      return;

    rawPwmPosSig.refresh();
    StatusCode status = rawPwmPosSig.getStatus();
    if (status == StatusCode.OK) {
      lastAbsTicks = angleRotToAbsTicks(rawPwmPosSig.getValueAsDouble());
      lastAbsRefreshTime = now;
    } else {
      // Do not spam errors here; just keep lastAbsTicks.
      // If you want visibility, you can publish status code:
      SmartDashboard.putString("Turret/AbsPwmStatus", status.toString());
      lastAbsRefreshTime = now; // still advance so we don't hammer CAN
    }
  }

  // ---------------- Public API ----------------

  /** Relative angle (degrees) where 0 = forward, CCW positive. */
  public double getAngleDeg() {
    return getPositionRot() * 360.0;
  }

  /** Relative position (rotations) where 0 = forward. */
  public double getPositionRot() {
    return positionSig.getValueAsDouble(); // rotations (assumes refreshed in periodic)
  }

  /** Relative velocity (rotations/sec). */
  public double getVelocityRps() {
    return velocitySig.getValueAsDouble(); // rotations/sec (assumes refreshed in periodic)
  }

  public double getVelocityDegPerSec() {
    return getVelocityRps() * 360.0;
  }

  public double getAppliedVolts() {
    return motorVoltageSig.getValueAsDouble(); // volts (assumes refreshed in periodic)
  }

  /** Cached absolute PWM ticks (0-4095). Refreshes slowly in periodic(). */
  public int getAbsoluteTicks() {
    return lastAbsTicks;
  }

  /** Sets whether goToAngle uses Motion Magic or simple position PID. */
  public void setUseMotionMagic(boolean enable) {
    useMotionMagic = enable;
  }

  /**
   * Open-loop manual control with safety clamp and software limits active on
   * device.
   */
  public void setDutyCycle(double duty) {
    duty = clamp(
        duty,
        -Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE,
        Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE);
    turret.setControl(dutyRequest.withOutput(duty));
  }

  public void stop() {
    turret.stopMotor();
  }

  /**
   * Go to desired turret angle relative to robot forward (device limits will
   * prevent exceeding).
   */
  public void goToAngleDeg(double targetDeg) {
    targetDeg = clamp(
        targetDeg,
        Constants.OperatorConstants.Turret.MIN_ANGLE_DEG,
        Constants.OperatorConstants.Turret.MAX_ANGLE_DEG);
    double targetRot = targetDeg / 360.0;

    if (useMotionMagic) {
      turret.setControl(motionMagicRequest.withPosition(targetRot));
    } else {
      turret.setControl(positionRequest.withPosition(targetRot));
    }
  }

  /** Returns true when turret is within tolerance (deg) of a target. */
  public boolean atAngleDeg(double targetDeg, double toleranceDeg) {
    return Math.abs(getAngleDeg() - targetDeg) <= toleranceDeg;
  }

  /** SysId factory commands (bind to buttons/dashboard). */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled())
      return new edu.wpi.first.wpilibj2.command.InstantCommand();
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled())
      return new edu.wpi.first.wpilibj2.command.InstantCommand();
    return sysIdRoutine.dynamic(direction);
  }

  // ---------------- SysId callbacks ----------------
  private void sysIdVoltageDrive(Voltage volts) {
    if (!isSysIdEnabled()) {
      stop();
      return;
    }
    double v = volts.in(Volts);
    double duty = v / RobotController.getBatteryVoltage();
    duty = clamp(
        duty,
        -Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE,
        Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE);
    setDutyCycle(duty);
  }

  private void sysIdLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled())
      return;
    log.motor("turret")
        .voltage(Volts.of(getAppliedVolts()))
        .angularPosition(Rotations.of(getPositionRot()))
        .angularVelocity(RotationsPerSecond.of(getVelocityRps()));
  }

  @Override
  public void periodic() {
    // Refresh the fast signals in one CAN batch
    BaseStatusSignal.refreshAll(positionSig, velocitySig, motorVoltageSig);

    // Refresh absolute PWM slowly (prevents -1003 spam)
    updateAbsoluteTicksOccasionally();

    SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
    SmartDashboard.putNumber("Turret/VelDegPerSec", getVelocityDegPerSec());
    SmartDashboard.putNumber("Turret/AppliedVolts", getAppliedVolts());
    SmartDashboard.putNumber("Turret/AbsTicks", getAbsoluteTicks());
    SmartDashboard.putNumber("Turret/RelRot", getPositionRot());
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim)
      return;

    final double dt = 0.02;

    var simState = turret.getSimState();

    // Keep CTRE sim supplied with the roboRIO voltage
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    // Drive physics sim using the voltage the controller is applying
    double appliedV = simState.getMotorVoltage();
    turretSim.setInputVoltage(appliedV);
    turretSim.update(dt);

    // FlywheelSim outputs rad/s. Convert to rotations/sec.
    double rps = turretSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Integrate position in rotations
    simPosRot += rps * dt;

    // Update the external sensor channels used by TalonFXS
    simState.setQuadratureVelocity(rps);
    simState.setRawQuadraturePosition(simPosRot);

    // PWM absolute should be in [0,1) rotations
    double pwmRot = simPosRot - Math.floor(simPosRot);
    simState.setPulseWidthPosition(pwmRot);

    // Battery sag approximation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDrawAmps()));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private static int wrapToHalfRev(int ticks, int modulus) {
    int half = modulus / 2;
    int t = ticks % modulus;
    if (t > half)
      t -= modulus;
    if (t < -half)
      t += modulus;
    return t;
  }
}
