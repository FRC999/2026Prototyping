package frc.robot.subsystems;

										  
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
													
import com.ctre.phoenix6.configs.MotorOutputConfigs;
															
import com.ctre.phoenix6.configs.Slot0Configs;
													   
import com.ctre.phoenix6.controls.DutyCycleOut;
													 
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
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
												   
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants.Turret;

/**
 * Turret using an absolute CAN Through-Bore (CANcoder) sensor (wraps every 360 deg) on a Talon FXS.
 *
 * <p>Key requirements (your rules):
 * <ul>
 *   <li>At boot, turret is within +/- 180 degrees of forward.</li>
 *   <li>Turret must never go beyond +/- 340 degrees of forward (umbilical safety).</li>
 *   <li>We track multi-turn angle in software (unwrap absolute) to enforce +/-340.</li>
 *   <li>We use hardware PID (PositionVoltage) but we dynamically toggle ContinuousWrap:
 *       <ul>
 *         <li>Wrap ON for "short way" moves (|delta| <= 180 deg)</li>
 *         <li>Wrap OFF when the short way would violate the +/-340 rule (forcing the long way)</li>
 *       </ul>
 *   </li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {

  // Turret motor controller on the specified CAN bus.
  private final TalonFXS turret =
      new TalonFXS(Turret.MOTOR_ID, Turret.CANBUS_NAME);

  // Absolute encoder (CAN Through-Bore / CANcoder) on the same CAN bus.
  private final CANcoder throughboreCANcoder = new CANcoder(Turret.CAN_ENCODER_ID, Turret.CANBUS_NAME);

  // Open-loop duty request (used for manual and SysId drive).
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  // Closed-loop position request (hardware position loop in Slot0).
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
																							  

  // Absolute CAN Through-Bore (0..1 rotations). (Wraps every revolution.)
  private final StatusSignal<Angle> absPosSig = throughboreCANcoder.getAbsolutePosition();
																				 
  // Motor voltage is used for telemetry and SysId logging.
  private final StatusSignal<Voltage> motorVoltageSig = turret.getMotorVoltage();

  // Used to guard sim-only code paths.
  private final boolean isSim = RobotBase.isSimulation();

  // If the absolute sensor only increases when turret turns CW, set +1 for CW-positive convention.
  // If you ever re-install and it flips, change to -1.
  private static final double ANGLE_SIGN = -1.0;

  // ---------------- Software unwrap tracking ----------------

  /** last wrapped absolute angle (deg) in [0, 360) */
  private double lastAbsDegWrapped = 0.0;

  /** continuous turret angle (deg), 0=forward, CCW positive, clamped to +/-340 */
  private double continuousDeg = 0.0;

  /** derived velocity estimate */
  private double lastContinuousDeg = 0.0;
  private double lastUpdateTs = Timer.getFPGATimestamp();
  private double estVelDegPerSec = 0.0;

  /** continuous target angle (deg) in [-340, +340] */
  private double targetDeg = 0.0;

  /** forward reference in degrees in the absolute sensor frame */
  private final double forwardDeg =
      (Constants.OperatorConstants.Turret.ABS_FORWARD_TICKS
          / (double) Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV) * 360.0;

  // ---------------- Continuous wrap toggling ----------------

  // Tracks the currently-applied wrap mode (so we don't spam configs).
  private boolean continuousWrapEnabled = true;

  // Prebuilt config objects for toggling wrap behavior.
  private final ClosedLoopGeneralConfigs clWrapOn = new ClosedLoopGeneralConfigs().withContinuousWrap(true);
  private final ClosedLoopGeneralConfigs clWrapOff = new ClosedLoopGeneralConfigs().withContinuousWrap(false);

  // ---------------- Simulation ----------------

  // Sim model used only in simulationPeriodic().
  private final FlywheelSim turretSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getVex775Pro(1),
              Constants.OperatorConstants.Turret.SIM_GEAR_RATIO,
              Constants.OperatorConstants.Turret.SIM_TURRET_J_KGM2),
          DCMotor.getVex775Pro(1));

  // Integrated simulated position in rotations.
  private double simPosRot = 0.0;

  // ---------------- SysId Characterization ----------------

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Seconds).of(Constants.OperatorConstants.SysId.TURRET_RAMP_RATE_V_PER_S),
              Volts.of(Constants.OperatorConstants.SysId.TURRET_STEP_V),
              Seconds.of(Constants.OperatorConstants.SysId.TURRET_TIMEOUT_S)),
          new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "turret"));

  /**
   * Runtime gating for SysId. Requires BOTH compile-time enable and dashboard enable.
													
   */
  private boolean isSysIdEnabled() {
    // Both gates must be true to allow SysId to actually move hardware.
    return Constants.OperatorConstants.SysId.ENABLE_SYSID
        && SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public TurretSubsystem() {
    // Hardware config: motor output + current limits + feedback + gains.
    configureHardware();

    // CAN signal update rates (reduces bus load but keeps control inputs fresh).
    configureStatusSignals();

    // Seed continuous angle from absolute on boot.
    seedFromAbsoluteAtBoot();

    // Dashboard defaults.
    SmartDashboard.putBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
    SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", continuousWrapEnabled);
  }

  private void configureStatusSignals() {			  
    // We want absolute angle to update quickly for unwrap math + control decisions.
    absPosSig.setUpdateFrequency(100.0);

    // Motor voltage can be slower; mostly for telemetry and SysId.
    motorVoltageSig.setUpdateFrequency(50.0);

    // Let Phoenix reduce unnecessary CAN chatter.
    turret.optimizeBusUtilization();
  }

  private InvertedValue motorInvertedValue() {			
																			  
    // Convert the project constant into Phoenix's inversion enum.
    return Constants.OperatorConstants.Turret.MOTOR_INVERTED
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
  }

  private SensorPhaseValue sensorPhaseValue() {
    // Sensor phase aligns the remote sensor's positive direction with the motor/controller convention.
																									 
    boolean inverted = Constants.OperatorConstants.Turret.SENSOR_PHASE_INVERTED;
    return inverted ? SensorPhaseValue.Opposed : SensorPhaseValue.Aligned;
  }

  private void configureHardware() {
    // Motor output (brake + inversion)
							
    MotorOutputConfigs out = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(motorInvertedValue());

    // Brushed commutation (you factory reset)
    CommutationConfigs commutation = new CommutationConfigs()
								
        .withMotorArrangement(MotorArrangementValue.Brushed_DC)
        .withBrushedMotorWiring(BrushedMotorWiringValue.Leads_A_and_B);

    // Current limits
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
								  
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LIMIT_A)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.OperatorConstants.Turret.STATOR_CURRENT_LIMIT_A);									   

    // Absolute Through-Bore via CAN (Remote CANcoder)
    // IMPORTANT: the sensor source must be RemoteCANcoder, and the CANcoder device must be assigned.
    ExternalFeedbackConfigs feedback = new ExternalFeedbackConfigs()
        .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.RemoteCANcoder)
        .withRemoteCANcoder(throughboreCANcoder)
												  
        .withSensorPhase(sensorPhaseValue());

    // Slot0 gains for hardware position loop (voltage-based in Phoenix 6)
    Slot0Configs slot0 = new Slot0Configs()
						  
        .withKP(Constants.OperatorConstants.Turret.kP)
        .withKI(Constants.OperatorConstants.Turret.kI)
        .withKD(Constants.OperatorConstants.Turret.kD)
        .withKS(Constants.OperatorConstants.Turret.kS)
        .withKV(Constants.OperatorConstants.Turret.kV)
        .withKA(Constants.OperatorConstants.Turret.kA);

    // Apply configs (each apply pushes config to the motor controller).
    turret.getConfigurator().apply(out);
    turret.getConfigurator().apply(commutation);
    turret.getConfigurator().apply(limits);
    turret.getConfigurator().apply(feedback);
    turret.getConfigurator().apply(slot0);								   

    // Default to wrap ON, and switch off when we must force long-way
    turret.getConfigurator().apply(clWrapOn);
    continuousWrapEnabled = true;
  }						 
											   
	public final double getAbsolutePosition() {
    // Absolute position from CANcoder in rotations [0,1) (wraps each revolution).
    return throughboreCANcoder.getAbsolutePosition().getValueAsDouble();
  }			  

  public final double getRelativePosition() {
    // Integrated/relative position from CANcoder in rotations (does not wrap in the same way).
    return throughboreCANcoder.getPosition().getValueAsDouble();
  }
								
  private void setContinuousWrap(boolean enable) {
    // If we're already in the desired wrap mode, do nothing (avoid CAN config spam).
    if (enable == continuousWrapEnabled) return;

    // Apply the wrap config to the motor controller.
    turret.getConfigurator().apply(enable ? clWrapOn : clWrapOff);

    // Record state and publish for debugging.
    continuousWrapEnabled = enable;
    SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", enable);
  }

  /**
   * Read absolute angle (deg) in [0, 360).
   * If signal is stale, returns last known value.					   
   */
  private double getAbsDegWrapped() {
    // Refresh the absolute signal before using it.
    absPosSig.refresh();

    // Capture status so we can see CAN dropouts / signal errors on dashboard.
    StatusCode status = absPosSig.getStatus();
    SmartDashboard.putString("Turret/AbsStatus", status.toString());

    // If not OK, keep last known value (prevents large jumps in unwrap logic).
    if (status != StatusCode.OK) {
      return lastAbsDegWrapped;
    }

    // Read absolute rotations (nominally [0,1) but we defensively wrap it anyway).
    double rot = absPosSig.getValueAsDouble();
    rot = rot - Math.floor(rot); // ensure [0,1)

    // Convert rotations to degrees.
    double deg = rot * 360.0;

    // Force degrees into [0,360) for stable delta math.
    return wrapTo0To360(deg);
  }

  /**
   * Seed software continuous angle at boot, assuming within +/-180 of forward.
   */
  private void seedFromAbsoluteAtBoot() {
    // Small delay to let CAN signals become valid right after startup.
    Timer.delay(0.05);

    // Get current absolute angle (wrapped [0,360)).
    double absDeg = getAbsDegWrapped();

    // Initialize last wrapped state for future delta calculations.
    lastAbsDegWrapped = absDeg;	

    // Compute shortest signed angle difference from the defined forward reference.
    // wrapToPlusMinus180 handles wrap-around at 0/360.
    double deltaDeg = ANGLE_SIGN * wrapToPlusMinus180(absDeg - forwardDeg);
										 
    // Boot assumption: within +/-180 (or whatever BOOT_MAX_ABS_DEG is set to).
    deltaDeg = clamp(
        deltaDeg,
        -Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG,
        Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG);

    // Initialize continuous turret position in your forward-relative frame.
    continuousDeg = deltaDeg;

    // Initialize velocity bookkeeping.
    lastContinuousDeg = continuousDeg;
														 
    // Initialize dt tracking so velocity math starts clean.
    lastUpdateTs = Timer.getFPGATimestamp();

    // Start target at current so there is no immediate step command.
    targetDeg = continuousDeg;
							 
    // Publish seed diagnostics to dashboard.
    SmartDashboard.putNumber("Turret/SeedAbsDeg", absDeg);
    SmartDashboard.putNumber("Turret/SeedContinuousDeg", continuousDeg);	 
  }

  /**
   * Update continuousDeg by unwrapping absolute position, clamping to +/-340.
																	   
   */
  private void updateContinuousAngle() {
    // Capture time and dt for velocity estimation.
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(1e-3, now - lastUpdateTs); // guard dt to avoid divide-by-zero

    // Current wrapped absolute angle.
    double absDeg = getAbsDegWrapped();

    // Smallest signed delta between successive wrapped absolute angles.
    double delta = ANGLE_SIGN * wrapToPlusMinus180(absDeg - lastAbsDegWrapped);

    // Apply delta to the multi-turn estimate.
    double nextContinuous = continuousDeg + delta;			

    // Hard safety clamp to +/- MAX (umbilical protection).
    nextContinuous = clamp(
        nextContinuous,
        Constants.OperatorConstants.Turret.MIN_ANGLE_DEG,
        Constants.OperatorConstants.Turret.MAX_ANGLE_DEG);

    // Compute deg/s based on change in continuous position.
    estVelDegPerSec = (nextContinuous - lastContinuousDeg) / dt;

    // Commit state for next loop iteration.
    continuousDeg = nextContinuous;
    lastContinuousDeg = continuousDeg;
    lastAbsDegWrapped = absDeg;
    lastUpdateTs = now;
  }

  // ---------------- Public API ----------------

  /** Continuous turret angle (deg), 0 = forward, CCW positive. */
  public double getAngleDeg() {
    // This is the software-unwrapped, safety-clamped turret angle.
    return continuousDeg;
  }

  public double getVelocityDegPerSec() {
    // Estimated velocity from continuous angle updates.
    return estVelDegPerSec;
  }

  public double getAppliedVolts() {
    // Refresh motor voltage signal (ensures telemetry reflects current output).
    motorVoltageSig.refresh();
    return motorVoltageSig.getValueAsDouble();
  }

  /** Absolute ticks (0-4095 equivalent) from wrapped absolute sensor. */
  public int getAbsoluteTicks() {
    // Convert last wrapped absolute degrees to a ticks-per-rev representation.
    double absDeg = lastAbsDegWrapped;

    // Scale degrees -> ticks and round to nearest int.
    int ticks = (int) Math.round((absDeg / 360.0) * Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV);

    // Wrap into [0, ticksPerRev).
    ticks %= Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;
    if (ticks < 0) ticks += Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;

    return ticks;
							
  }

  /** Open-loop manual control with safety clamp. */
  public void setDutyCycle(double duty) {
    // Clamp duty to avoid commanding beyond your configured safe range.
    duty = clamp(
			  
        duty,
        -Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE,
        Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE);

    // Send open-loop command to the motor controller.
    turret.setControl(dutyRequest.withOutput(duty));
  }

  public void stop() {
    // Immediately stop output.
    turret.stopMotor();
  }

  /** Current continuous turret angle in degrees in this subsystem's reference frame (0 = "forward" per ABS_FORWARD_TICKS, CCW+). */
  public double getContinuousAngleDeg() {
    return continuousDeg;
  }

  /** Estimated turret angular velocity in deg/sec (sign matches getContinuousAngleDeg convention). */
  public double getEstimatedVelocityDegPerSec() {
    return estVelDegPerSec;
  }

  /**
   * Command turret to desired angle (deg) relative to forward.
   *
   * <p>This method enforces +/-340 safety and chooses the best equivalent target among
   * {deg, deg+360, deg-360} that stays inside the allowed range and minimizes travel.
   *
   * <p>Then it toggles continuous wrap:
   *  - Wrap ON if |delta| <= 180 (short way)
   *  - Wrap OFF if |delta| > 180 (force long way)
   */
  public void goToAngleDeg(double desiredDeg) {
    // Find the closest safe equivalent of desiredDeg (handles ±360 wrap candidates).
    double best = chooseBestEquivalentTargetDeg(desiredDeg);

    // If no safe target exists, refuse the command and report.
    if (Double.isNaN(best)) {
      stop();
      SmartDashboard.putString("Turret/GoalStatus", "REJECTED_UNREACHABLE");
      return;
    }

    // Store chosen target and compute delta from current continuous position.
    targetDeg = best;
    double delta = targetDeg - continuousDeg;

    // If short-way motion is <= 180°, enable continuous wrap; else force long-way.
    boolean wantWrap = Math.abs(delta) <= 180.0;
    setContinuousWrap(wantWrap);

    // Convert continuous target to wrapped [0,1) rotations in encoder frame.
    // Multiply by ANGLE_SIGN so the controller sees the correct sign convention.
    double targetRot = ANGLE_SIGN * wrappedRotFromContinuousDeg(targetDeg);

    // Command the hardware position loop.
    turret.setControl(positionRequest.withPosition(targetRot));

    // Telemetry block: target selection and mode decisions.
    SmartDashboard.putNumber("Turret/TargetDeg", targetDeg);
    SmartDashboard.putNumber("Turret/DeltaDegCmd", delta);
    SmartDashboard.putString("Turret/GoalStatus", wantWrap ? "SHORT_WRAP_ON" : "LONG_WRAP_OFF");
  }

  /** true if turret is within tolerance of desired angle (deg), using best safe equivalent. */
  public boolean atAngleDeg(double desiredDeg, double toleranceDeg) {
    // Compute the safe equivalent target we would command.
    double best = chooseBestEquivalentTargetDeg(desiredDeg);

    // If unreachable safely, then we can't be "at" it.
    if (Double.isNaN(best)) return false;

    // Compare current continuous position to the best safe target.
    return Math.abs(continuousDeg - best) <= toleranceDeg;
  }

  /**
   * Choose best equivalent target among {deg, deg+360, deg-360} that:
   *  - stays within [MIN_ANGLE_DEG, MAX_ANGLE_DEG]
   *  - minimizes travel from current continuousDeg
   */
  private double chooseBestEquivalentTargetDeg(double desiredDeg) {
    // Pull safety range from constants.
    double min = Constants.OperatorConstants.Turret.MIN_ANGLE_DEG;
    double max = Constants.OperatorConstants.Turret.MAX_ANGLE_DEG;

    // Clamp requested target into legal range first (keeps intent sane).
    desiredDeg = clamp(desiredDeg, min, max);

    // Consider equivalent angles one revolution away.
    double[] candidates = new double[] { desiredDeg, desiredDeg + 360.0, desiredDeg - 360.0 };

    // Track the best (closest) candidate within legal range.
    double best = Double.NaN;
    double bestDist = Double.POSITIVE_INFINITY;

    // Search candidates for the closest safe move.
    for (double c : candidates) {
      // Skip candidates that violate +/-340 hard limits.
      if (c < min || c > max) continue;

      // Distance is evaluated in continuous space (deg).
      double dist = Math.abs(c - continuousDeg);

      // Keep the closest.
      if (dist < bestDist) {
        bestDist = dist;
        best = c;
      }
    }

    return best;
  }

  /**
   * Convert a continuous target (deg relative to forward) into the wrapped absolute
   * rotation value [0,1).
   */
  private double wrappedRotFromContinuousDeg(double continuousDegTarget) {
    // Convert from forward-relative degrees back into encoder-frame absolute degrees.
    double absDeg = forwardDeg + (continuousDegTarget);

    // Wrap to [0,360) so we can convert to a wrapped rotation value.
    absDeg = wrapTo0To360(absDeg);

    // Convert degrees to rotations [0,1).
    return absDeg / 360.0;
  }

  // ---------------- SysId commands ----------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // If not enabled, return a no-op command so nothing moves.
    if (!isSysIdEnabled()) return new edu.wpi.first.wpilibj2.command.InstantCommand();

    // Otherwise run SysId's quasistatic ramp.
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // If not enabled, return a no-op command so nothing moves.
    if (!isSysIdEnabled()) return new edu.wpi.first.wpilibj2.command.InstantCommand();

    // Otherwise run SysId's dynamic step.
    return sysIdRoutine.dynamic(direction);
  }

  // ---------------- SysId callbacks ----------------

  private void sysIdVoltageDrive(Voltage volts) {
    // Safety: if SysId isn't enabled, ensure turret is stopped.
    if (!isSysIdEnabled()) {
      stop();
      return;
    }

    // Convert requested voltage to a duty cycle relative to current battery voltage.
    double v = volts.in(Volts);
    double duty = v / RobotController.getBatteryVoltage();

    // Clamp duty to match your max output limits.
    duty = clamp(
			  
        duty,
        -Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE,
        Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE);

    // Apply duty request to the motor controller.
    setDutyCycle(duty);
  }

  private void sysIdLog(SysIdRoutineLog log) {
    // Only log when SysId is enabled.
    if (!isSysIdEnabled()) return;

    // Log standard motor signals for SysId analysis.
    log.motor("turret")
        .voltage(Volts.of(getAppliedVolts()))
        .angularPosition(Rotations.of(getAngleDeg() / 360.0))
        .angularVelocity(RotationsPerSecond.of(getVelocityDegPerSec() / 360.0));
  }

  @Override
  public void periodic() {
    // Update continuous (multi-turn) angle state every loop.
    updateContinuousAngle();

    // Telemetry block: expose key state for debugging and tuning.
    SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
    SmartDashboard.putNumber("Turret/VelDegPerSec", getVelocityDegPerSec());
    SmartDashboard.putNumber("Turret/AppliedVolts", getAppliedVolts());
    SmartDashboard.putNumber("Turret/AbsTicks", getAbsoluteTicks());
    SmartDashboard.putNumber("Turret/AbsDegWrapped", lastAbsDegWrapped);
    SmartDashboard.putNumber("Turret/TargetDeg", targetDeg);
    SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", continuousWrapEnabled);
  }

  @Override
  public void simulationPeriodic() {
    // Only run simulation when in sim.
    if (!isSim) return;

    // Fixed timestep for sim integration.
    final double dt = 0.02;

    // Read/drive Talon simulated state.
    var simState = turret.getSimState();
												  
    // Provide simulated supply voltage from RoboRIO sim.
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());
											 
    // Applied motor voltage from controller.
    double appliedV = simState.getMotorVoltage();

    // Feed into the physics model.
    turretSim.setInputVoltage(appliedV);
    turretSim.update(dt);
														   
    // Convert rad/s -> rotations/s for integrating encoder position.
    double rps = turretSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
									  
    // Integrate simulated position in rotations.
    simPosRot += rps * dt;					

    // Feed the simulated position/velocity into the CAN Through-Bore (CANcoder)
    var encoderSimState = throughboreCANcoder.getSimState();
    encoderSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    encoderSimState.setRawPosition(simPosRot);
    encoderSimState.setVelocity(rps);
								
    // Battery voltage sag simulation based on current draw.
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDrawAmps()));
  }

  // ---------------- Helpers ----------------

  private static double clamp(double v, double lo, double hi) {
    // Clamp v into [lo, hi].
    return Math.max(lo, Math.min(hi, v));
  }

  private static double wrapTo0To360(double deg) {
    // Wrap any degrees into [0,360).
    double d = deg % 360.0;
    if (d < 0) d += 360.0;
    return d;
  }

  /** wrap to (-180, 180] */
  private static double wrapToPlusMinus180(double deg) {
    // Wrap degrees into (-180,180] to compute shortest signed difference.
    double d = ((deg + 180.0) % 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
  }


// ---------------------------------------------------------------------------
// Hood placeholders (your hood hardware is not yet implemented in this repo)
// ---------------------------------------------------------------------------
/** Last hood command (rad). This is a placeholder until the hood motor/encoder is implemented. */
private double hoodCommandRad = 0.0;

/**
 * Command the hood to an angle (radians).
 *
 * <p>Placeholder:
 * - Store the setpoint for telemetry and for solver "closest solution" heuristics.
 * - Implement motor control + encoder feedback later.
 */
public void setHoodAngleRad(double hoodAngleRad) {
  hoodCommandRad = hoodAngleRad;
}

/** @return last hood setpoint in radians (placeholder). */
public double getHoodCommandRad() {
  return hoodCommandRad;
}

}
