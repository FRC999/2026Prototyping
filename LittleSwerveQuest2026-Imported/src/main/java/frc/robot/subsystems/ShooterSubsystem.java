package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants;

/** Kraken X60 shooter prototype (TalonFX, Phoenix 6). */
public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooter = new TalonFX(Constants.OperatorConstants.Shooter.CAN_ID,
      Constants.OperatorConstants.Shooter.CANBUS_NAME);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  private double targetRpm = 0.0;
  private double lastRpm = 0.0;
  private boolean wasReady = false;
  private double readySince = 0.0;
  private boolean dipDetected = false;

  // ---------------- Shooter readiness (rolling window stats) ----------------
  private final double[] rpmWindow = new double[Constants.OperatorConstants.Shooter.READY_WINDOW_SAMPLES];
  private int rpmWindowCount = 0;
  private int rpmWindowIndex = 0;

  private double rpmMean = 0.0;
  private double rpmStdDev = 0.0;

  private final boolean isSim = RobotBase.isSimulation();

  // WPILib 2026 FlywheelSim uses a plant + motor model
  private final FlywheelSim flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          Constants.OperatorConstants.Shooter.SIM_GEAR_RATIO,
          Constants.OperatorConstants.Shooter.SIM_J_KGM2),
      DCMotor.getKrakenX60(1));

  // Phoenix 6 typed signals
  private final StatusSignal<AngularVelocity> velocitySig = shooter.getVelocity();
  private final StatusSignal<Voltage> motorVoltageSig = shooter.getMotorVoltage();

  // ---------------- SysId Characterization ----------------
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Seconds).of(Constants.OperatorConstants.SysId.SHOOTER_RAMP_RATE_V_PER_S),
          Volts.of(Constants.OperatorConstants.SysId.SHOOTER_STEP_V),
          Seconds.of(Constants.OperatorConstants.SysId.SHOOTER_TIMEOUT_S)),
      new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "shooter"));

  /** Runtime gating for SysId. */
  private boolean isSysIdEnabled() {
    return Constants.OperatorConstants.SysId.ENABLE_SYSID
        && SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public ShooterSubsystem() {
    configureHardware();
    configureStatusSignals();
  }

  private void configureStatusSignals() {
    velocitySig.setUpdateFrequency(100.0); // 100
    motorVoltageSig.setUpdateFrequency(100.0); // 50
    shooter.optimizeBusUtilization();
  }

  private void configureHardware() {
    MotorOutputConfigs out = new MotorOutputConfigs()
        .withNeutralMode(
            Constants.OperatorConstants.Shooter.NEUTRAL_COAST
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake)
        .withInverted(
            Constants.OperatorConstants.Shooter.MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.OperatorConstants.Shooter.SUPPLY_CURRENT_LIMIT_A)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.OperatorConstants.Shooter.STATOR_CURRENT_LIMIT_A);

    Slot0Configs slot0 = new Slot0Configs()
        .withKP(Constants.OperatorConstants.Shooter.kP)
        .withKI(Constants.OperatorConstants.Shooter.kI)
        .withKD(Constants.OperatorConstants.Shooter.kD)
        .withKS(Constants.OperatorConstants.Shooter.kS)
        .withKV(Constants.OperatorConstants.Shooter.kV)
        .withKA(Constants.OperatorConstants.Shooter.kA);

    TalonFXConfiguration cfg = new TalonFXConfiguration().withMotorOutput(out).withCurrentLimits(limits)
        .withSlot0(slot0);

    shooter.getConfigurator().apply(cfg);
  }

  // ---------------- Public API ----------------

  /** Target shooter speed (RPM). Uses hardware velocity control. */
  public void setTargetRpm(double rpm) {
    targetRpm = Math.max(0.0, rpm);
    dipDetected = false;
    readySince = 0.0;
    wasReady = false;

    double targetRps = targetRpm / 60.0; // Phoenix 6 uses rotations/sec
    shooter.setControl(velocityRequest.withVelocity(targetRps));
  }

  /** Open-loop duty-cycle (for quick tests). */
  public void setDutyCycle(double duty) {
    duty = clamp(
        duty,
        -Constants.OperatorConstants.Shooter.MAX_DUTY_CYCLE,
        Constants.OperatorConstants.Shooter.MAX_DUTY_CYCLE);
    targetRpm = 0.0;
    shooter.setControl(dutyRequest.withOutput(duty));
  }

  public void stop() {
    targetRpm = 0.0;
    shooter.stopMotor();
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  /** Shooter velocity in rotations/sec. */
  public double getVelocityRps() {
    return velocitySig.getValueAsDouble();
  }

  public double getVelocityRpm() {
    return getVelocityRps() * 60.0;
  }

  public double getAppliedVolts() {
    return motorVoltageSig.getValueAsDouble();
  }

  /** "Power" as a duty-cycle estimate (applied volts / battery volts). */
  public double getAppliedDuty() {
    double batt = RobotController.getBatteryVoltage();
    if (batt <= 1e-6)
      return 0.0;
    return getAppliedVolts() / batt;
  }

  /** True when shooter has been within tolerance for READY_MIN_TIME_S. */
  public boolean isReadyToShoot() {
    return wasReady;
  }

  /**
   * True when we observed a speed dip after being ready (proxy for a ball hit).
   */
  public boolean wasDipDetected() {
    return dipDetected;
  }

  // ---------------- SysId factory commands ----------------
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
  private void sysIdVoltageDrive(edu.wpi.first.units.measure.Voltage volts) {
    if (!isSysIdEnabled()) {
      stop();
      return;
    }
    double v = volts.in(Volts);

    double duty = v / RobotController.getBatteryVoltage();
    duty = clamp(
        duty,
        -Constants.OperatorConstants.Shooter.MAX_DUTY_CYCLE,
        Constants.OperatorConstants.Shooter.MAX_DUTY_CYCLE);

    shooter.setControl(dutyRequest.withOutput(duty));
  }

  private void sysIdLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled())
      return;
    log.motor("shooter")
        .voltage(Volts.of(getAppliedVolts()))
        .angularPosition(Rotations.of(0.0))
        .angularVelocity(RotationsPerSecond.of(getVelocityRps()));
  }

  @Override
  public void periodic() {
    // refresh fast signals as a batch
    BaseStatusSignal.refreshAll(velocitySig, motorVoltageSig);

    double rpm = getVelocityRpm();

    /*
     * // Ready logic PREVIOUS
     * boolean inTol = targetRpm > 1.0
     * && Math.abs(rpm - targetRpm) <=
     * Constants.OperatorConstants.Shooter.READY_TOLERANCE_RPM;
     * 
     * double now = Timer.getFPGATimestamp();
     * if (inTol) {
     * if (readySince <= 0.0)
     * readySince = now;
     * if (!wasReady && (now - readySince) >=
     * Constants.OperatorConstants.Shooter.READY_MIN_TIME_S) {
     * wasReady = true;
     * }
     * } else {
     * readySince = 0.0;
     * wasReady = false;
     * }
     */

    // Ready logic (windowed mean/stddev)
    updateReadinessStats(rpm);

    boolean readyNow = targetRpm > 1.0
        && Math.abs(rpmMean - targetRpm) <= Constants.OperatorConstants.Shooter.READY_RPM_TOLERANCE
        && rpmStdDev <= Constants.OperatorConstants.Shooter.READY_STDDEV_MAX;

    // Optional: keep your existing READY_MIN_TIME_S behavior *on top* of windowed
    // ready
    double now = Timer.getFPGATimestamp();
    if (readyNow) {
      if (readySince <= 0.0)
        readySince = now;
      if (!wasReady && (now - readySince) >= Constants.OperatorConstants.Shooter.READY_MIN_TIME_S) {
        wasReady = true;
      }
    } else {
      readySince = 0.0;
      wasReady = false;
    }

    // Dip detection
    if (!dipDetected && wasReady && (lastRpm - rpm) >= Constants.OperatorConstants.Shooter.DIP_DETECT_DROP_RPM) {
      dipDetected = true;
    }
    lastRpm = rpm;

    // Dashboard
    SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
    SmartDashboard.putNumber("Shooter/RPM", rpm);
    SmartDashboard.putBoolean("Shooter/Ready", wasReady);
    SmartDashboard.putBoolean("Shooter/DipDetected", dipDetected);

    SmartDashboard.putNumber("Shooter/AppliedVolts", getAppliedVolts());
    SmartDashboard.putNumber("Shooter/AppliedDuty", getAppliedDuty());
    SmartDashboard.putNumber("Shooter/BatteryVolts", RobotController.getBatteryVoltage());
  }

  private void updateReadinessStats(double rpm) {
    rpmWindow[rpmWindowIndex] = rpm;
    rpmWindowIndex = (rpmWindowIndex + 1) % rpmWindow.length;
    if (rpmWindowCount < rpmWindow.length)
      rpmWindowCount++;

    if (rpmWindowCount < rpmWindow.length) {
      rpmMean = rpm;
      rpmStdDev = 999.0;
      wasReady = false;
      return;
    }

    double sum = 0.0;
    for (double v : rpmWindow)
      sum += v;
    rpmMean = sum / rpmWindow.length;

    double var = 0.0;
    for (double v : rpmWindow) {
      double d = v - rpmMean;
      var += d * d;
    }
    rpmStdDev = Math.sqrt(var / rpmWindow.length);

    SmartDashboard.putNumber("Shooter/RPM_Mean200ms", rpmMean);
    SmartDashboard.putNumber("Shooter/RPM_StdDev200ms", rpmStdDev);

  }

  @Override
  public void simulationPeriodic() {
    if (!isSim)
      return;

    final double dt = 0.02;

    var simState = shooter.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    double appliedV = simState.getMotorVoltage();
    flywheelSim.setInputVoltage(appliedV);
    flywheelSim.update(dt);

    double rps = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    simState.setRotorVelocity(rps);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
