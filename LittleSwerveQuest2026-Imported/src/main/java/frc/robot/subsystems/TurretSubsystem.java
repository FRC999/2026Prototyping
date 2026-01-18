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

/**
 * Turret using ONLY an absolute PWM sensor (wraps every 360 deg) on a Talon FXS.
 *
 * <p>Key requirements (your rules):
 * <ul>
 *   <li>At boot, turret is within +/- 180 degrees of forward.</li>
 *   <li>Turret must never go beyond +/- 340 degrees of forward (umbilical safety).</li>
 *   <li>We track multi-turn angle in software (unwrap PWM) to enforce +/-340.</li>
 *   <li>We use hardware PID (PositionVoltage) but we dynamically toggle ContinuousWrap:
 *       <ul>
 *         <li>Wrap ON for "short way" moves (|delta| <= 180 deg)</li>
 *         <li>Wrap OFF when the short way would violate the +/-340 rule (forcing the long way)</li>
 *       </ul>
 *   </li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {

  private final TalonFXS turret =
      new TalonFXS(Constants.OperatorConstants.Turret.CAN_ID, Constants.OperatorConstants.Turret.CANBUS_NAME);

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
																							  

  // PWM absolute (0..1 rotations)
  private final StatusSignal<Angle> rawPwmPosSig = turret.getRawPulseWidthPosition();
																				 
  private final StatusSignal<Voltage> motorVoltageSig = turret.getMotorVoltage();

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

  /** forward reference in degrees in the PWM frame */
  private final double forwardDeg =
      (Constants.OperatorConstants.Turret.ABS_FORWARD_TICKS
          / (double) Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV) * 360.0;

  // ---------------- Continuous wrap toggling ----------------

  private boolean continuousWrapEnabled = true;
  private final ClosedLoopGeneralConfigs clWrapOn = new ClosedLoopGeneralConfigs().withContinuousWrap(true);
  private final ClosedLoopGeneralConfigs clWrapOff = new ClosedLoopGeneralConfigs().withContinuousWrap(false);

  // ---------------- Simulation ----------------

  private final FlywheelSim turretSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getVex775Pro(1),
              Constants.OperatorConstants.Turret.SIM_GEAR_RATIO,
              Constants.OperatorConstants.Turret.SIM_TURRET_J_KGM2),
          DCMotor.getVex775Pro(1));

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
    return Constants.OperatorConstants.SysId.ENABLE_SYSID
        && SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public TurretSubsystem() {
    configureHardware();
    configureStatusSignals();
    seedFromAbsoluteAtBoot();

    SmartDashboard.putBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
    SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", continuousWrapEnabled);
  }

  private void configureStatusSignals() {
													 
										  
    rawPwmPosSig.setUpdateFrequency(100.0);
    motorVoltageSig.setUpdateFrequency(50.0);

																			
										  

    turret.optimizeBusUtilization();
  }

										  
															   
																   
								  
								   
   

  private InvertedValue motorInvertedValue() {
										
																			  
    return Constants.OperatorConstants.Turret.MOTOR_INVERTED
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
  }

  private SensorPhaseValue sensorPhaseValue() {
    // You said you changed sensor phase; keep a constant for this if you have it.
    // If you do NOT have a constant, replace this with Aligned and handle phase via motor inversion.
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

								  
																	
																							  

						   
						   
								
																								
																						   

    // PWM feedback (absolute)
    ExternalFeedbackConfigs pwm = new ExternalFeedbackConfigs()
        .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.PulseWidth)
        // Important: makes wrap behavior symmetric around 0.5 rotations.
        .withAbsoluteSensorDiscontinuityPoint(0.5)
        .withSensorPhase(sensorPhaseValue());

    // Slot0 gains for hardware position loop (voltage-based in Phoenix 6)
    Slot0Configs slot0 = new Slot0Configs()
						  
        .withKP(Constants.OperatorConstants.Turret.kP)
        .withKI(Constants.OperatorConstants.Turret.kI)
        .withKD(Constants.OperatorConstants.Turret.kD)
        .withKS(Constants.OperatorConstants.Turret.kS)
        .withKV(Constants.OperatorConstants.Turret.kV)
        .withKA(Constants.OperatorConstants.Turret.kA);

    // Apply configs
    turret.getConfigurator().apply(out);
    turret.getConfigurator().apply(commutation);
    turret.getConfigurator().apply(limits);
    turret.getConfigurator().apply(pwm);
    turret.getConfigurator().apply(slot0);
												  
											 
												   

    // Default to wrap ON, and switch off when we must force long-way
    turret.getConfigurator().apply(clWrapOn);
    continuousWrapEnabled = true;
  }
									  
										 
											   
										  
								
							  

  private void setContinuousWrap(boolean enable) {
    if (enable == continuousWrapEnabled) return;
    turret.getConfigurator().apply(enable ? clWrapOn : clWrapOff);
    continuousWrapEnabled = enable;
    SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", enable);
  }

  /**
   * Read absolute PWM angle (deg) in [0, 360).
   * If signal is stale, returns last known value.
	
																			 
																				 
																   
   */
  private double getAbsDegWrapped() {
    rawPwmPosSig.refresh();
    StatusCode status = rawPwmPosSig.getStatus();
    SmartDashboard.putString("Turret/AbsPwmStatus", status.toString());
																						   
													  
											

																			 
										

    if (status != StatusCode.OK) {
      return lastAbsDegWrapped;
    }

    double rot = rawPwmPosSig.getValueAsDouble();
    rot = rot - Math.floor(rot);
    double deg = rot * 360.0;
    return wrapTo0To360(deg);
  }

  /**
   * Seed software continuous angle at boot, assuming within +/-180 of forward.
   */
  private void seedFromAbsoluteAtBoot() {
    Timer.delay(0.05);

    double absDeg = getAbsDegWrapped();
    lastAbsDegWrapped = absDeg;
									 
																						   
																								 
											

    double deltaDeg = ANGLE_SIGN * wrapToPlusMinus180(absDeg - forwardDeg);
										 

    // Boot assumption: within +/-180
    deltaDeg = clamp(
        deltaDeg,
        -Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG,
        Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG);

    continuousDeg = deltaDeg;
    lastContinuousDeg = continuousDeg;
								   
	 
														 
    lastUpdateTs = Timer.getFPGATimestamp();
    targetDeg = continuousDeg;
							 
												   
									
																   
	   
						
	 
									
   

    SmartDashboard.putNumber("Turret/SeedAbsDeg", absDeg);
    SmartDashboard.putNumber("Turret/SeedContinuousDeg", continuousDeg);
										 
																							 
																  
																				 
				 
  }

  /**
   * Update continuousDeg by unwrapping PWM absolute, clamping to +/-340.
																	   
   */
  private void updateContinuousAngle() {
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(1e-3, now - lastUpdateTs);

    double absDeg = getAbsDegWrapped();
    double delta = ANGLE_SIGN * wrapToPlusMinus180(absDeg - lastAbsDegWrapped);

    double nextContinuous = continuousDeg + delta;
																		 
							   
															
			
															   
																		 
							   
	 
   

    // Hard safety clamp to +/- MAX
    nextContinuous = clamp(
        nextContinuous,
        Constants.OperatorConstants.Turret.MIN_ANGLE_DEG,
        Constants.OperatorConstants.Turret.MAX_ANGLE_DEG);

    estVelDegPerSec = (nextContinuous - lastContinuousDeg) / dt;
							   
									
   

    continuousDeg = nextContinuous;
    lastContinuousDeg = continuousDeg;
    lastAbsDegWrapped = absDeg;
    lastUpdateTs = now;
  }

  // ---------------- Public API ----------------

  /** Continuous turret angle (deg), 0 = forward, CCW positive. */
  public double getAngleDeg() {
    return continuousDeg;
  }

  public double getVelocityDegPerSec() {
    return estVelDegPerSec;
  }

  public double getAppliedVolts() {
    motorVoltageSig.refresh();
    return motorVoltageSig.getValueAsDouble();
  }

  /** Absolute ticks (0-4095 equivalent) from wrapped PWM. */
  public int getAbsoluteTicks() {
    double absDeg = lastAbsDegWrapped;
    int ticks = (int) Math.round((absDeg / 360.0) * Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV);
    ticks %= Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;
    if (ticks < 0) ticks += Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;
    return ticks;
							
  }

  /** Open-loop manual control with safety clamp. */
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
    double best = chooseBestEquivalentTargetDeg(desiredDeg);
			  
					  
															 
															  
										 

    if (Double.isNaN(best)) {
      stop();
      SmartDashboard.putString("Turret/GoalStatus", "REJECTED_UNREACHABLE");
      return;
    }

    targetDeg = best;
    double delta = targetDeg - continuousDeg;

    boolean wantWrap = Math.abs(delta) <= 180.0;
    setContinuousWrap(wantWrap);

    double targetRot = ANGLE_SIGN * wrappedRotFromContinuousDeg(targetDeg);

    turret.setControl(positionRequest.withPosition(targetRot));

    SmartDashboard.putNumber("Turret/TargetDeg", targetDeg);
    SmartDashboard.putNumber("Turret/DeltaDegCmd", delta);
    SmartDashboard.putString("Turret/GoalStatus", wantWrap ? "SHORT_WRAP_ON" : "LONG_WRAP_OFF");
  }

  /** true if turret is within tolerance of desired angle (deg), using best safe equivalent. */
  public boolean atAngleDeg(double desiredDeg, double toleranceDeg) {
    double best = chooseBestEquivalentTargetDeg(desiredDeg);
    if (Double.isNaN(best)) return false;
    return Math.abs(continuousDeg - best) <= toleranceDeg;
  }

  /**
   * Choose best equivalent target among {deg, deg+360, deg-360} that:
   *  - stays within [MIN_ANGLE_DEG, MAX_ANGLE_DEG]
   *  - minimizes travel from current continuousDeg
   */
  private double chooseBestEquivalentTargetDeg(double desiredDeg) {
    double min = Constants.OperatorConstants.Turret.MIN_ANGLE_DEG;
    double max = Constants.OperatorConstants.Turret.MAX_ANGLE_DEG;

    // clamp base desired into allowable range first (keeps intent sane)
    desiredDeg = clamp(desiredDeg, min, max);

    double[] candidates = new double[] { desiredDeg, desiredDeg + 360.0, desiredDeg - 360.0 };

    double best = Double.NaN;
    double bestDist = Double.POSITIVE_INFINITY;

    for (double c : candidates) {
      if (c < min || c > max) continue;
      double dist = Math.abs(c - continuousDeg);
      if (dist < bestDist) {
        bestDist = dist;
        best = c;
      }
    }
    return best;
  }

  /**
   * Convert a continuous target (deg relative to forward) into the wrapped PWM
   * rotation value [0,1).
   */
  private double wrappedRotFromContinuousDeg(double continuousDegTarget) {
    double absDeg = forwardDeg + (continuousDegTarget);
    absDeg = wrapTo0To360(absDeg);
    return absDeg / 360.0;
  }

  // ---------------- SysId commands ----------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) return new edu.wpi.first.wpilibj2.command.InstantCommand();
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) return new edu.wpi.first.wpilibj2.command.InstantCommand();
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
    if (!isSysIdEnabled()) return;
    log.motor("turret")
        .voltage(Volts.of(getAppliedVolts()))
        .angularPosition(Rotations.of(getAngleDeg() / 360.0))
        .angularVelocity(RotationsPerSecond.of(getVelocityDegPerSec() / 360.0));
  }

  @Override
  public void periodic() {
    updateContinuousAngle();
																		   

														
									  

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
    if (!isSim) return;

    final double dt = 0.02;

    var simState = turret.getSimState();

												  
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

																	 
    double appliedV = simState.getMotorVoltage();
    turretSim.setInputVoltage(appliedV);
    turretSim.update(dt);

														   
    double rps = turretSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

									  
    simPosRot += rps * dt;

														   
										
												 

												
    double pwmRot = simPosRot - Math.floor(simPosRot);
    simState.setPulseWidthPosition(pwmRot);

								
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDrawAmps()));
  }

  // ---------------- Helpers ----------------

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private static double wrapTo0To360(double deg) {
    double d = deg % 360.0;
    if (d < 0) d += 360.0;
    return d;
  }

  /** wrap to (-180, 180] */
  private static double wrapToPlusMinus180(double deg) {
    double d = ((deg + 180.0) % 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
  }
}
