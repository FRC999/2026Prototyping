package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class ShooterSubsystem extends SubsystemBase {
  public enum ShooterState {
    IDLE,
    SPINNING_UP,
    AT_SPEED,
    DIP_DETECTED,
    RECOVERING
  }

  private final TalonFX shooter = new TalonFX(Constants.Shooter.KRAKEN_CAN_ID);

  private double targetRpm = Constants.Shooter.DEFAULT_RPM;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

  // State machine
  private ShooterState state = ShooterState.IDLE;
  private boolean ready = false;

  private double stableTime = 0.0;
  private double dipTime = 0.0;
  private double recoverStableTime = 0.0;

  private double lastDipMagnitudeRpm = 0.0;
  private double lastDipMinRpm = 0.0;
  private double lastRecoveryTimeS = 0.0;
  private int shotCount = 0;

  // For accel estimate (filtered)
  private double lastVelRpm = 0.0;
  private double accelRpmPerSec = 0.0;

  private final boolean isSim = RobotBase.isSimulation();

  private final FlywheelSim flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),   // gearbox/motor model
          1.0,                       // gearing (1:1)
          Constants.Shooter.SIM_J_KGM2 // moment of inertia kg*m^2
      ),
      DCMotor.getKrakenX60(1)        // gearbox again (used for current draw calc)
  );
  private double simRotorPositionRot = 0.0;

  private double simRotorPosRad = 0.0;


  public ShooterSubsystem() {
    configureKraken();
  }

  private void configureKraken() {
  MotorOutputConfigs out = new MotorOutputConfigs()
      .withInverted(Constants.Shooter.INVERTED)
      .withNeutralMode(Constants.Shooter.NEUTRAL_MODE);

  CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(Constants.Shooter.ENABLE_CURRENT_LIMIT)
      .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT_A)
      .withSupplyCurrentLowerTime(Constants.Shooter.SUPPLY_CURRENT_LOWER_TIME_S)
      .withSupplyCurrentLowerLimit(Constants.Shooter.SUPPLY_CURRENT_LOWER_LIMIT_A)
      .withStatorCurrentLimitEnable(Constants.Shooter.ENABLE_CURRENT_LIMIT)
      .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT_A);

  Slot0Configs slot0 = new Slot0Configs()
      .withKP(Constants.Shooter.kP)
      .withKI(Constants.Shooter.kI)
      .withKD(Constants.Shooter.kD)
      .withKV(Constants.Shooter.kV);

  TalonFXConfiguration cfg = new TalonFXConfiguration()
      .withMotorOutput(out)
      .withCurrentLimits(limits)
      .withSlot0(slot0);

  shooter.getConfigurator().apply(cfg);
}

  // ---------------- Controls ----------------

  public void stop() {
    shooter.setControl(dutyCycle.withOutput(0.0));

    state = ShooterState.IDLE;
    ready = false;
    stableTime = dipTime = recoverStableTime = 0.0;
  }

  public void setTargetRpm(double rpm) {
    targetRpm = clamp(rpm, 0.0, Constants.Shooter.MAX_RPM);
  }

  public void adjustTargetRpm(double delta) {
    setTargetRpm(targetRpm + delta);
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  public void setDutyCycle(double percent) {
    double p = clamp(percent, -Constants.Shooter.MAX_DUTY_CYCLE, Constants.Shooter.MAX_DUTY_CYCLE);
    shooter.setControl(dutyCycle.withOutput(p));
  }

  public void setVoltage(double volts) {
    double v = clamp(volts, -Constants.Shooter.MAX_VOLTS, Constants.Shooter.MAX_VOLTS);
    shooter.setControl(voltageOut.withOutput(v));
  }

  public void runVelocityPIDRpm(double rpm) {
    double safeRpm = clamp(rpm, 0.0, Constants.Shooter.MAX_RPM);
    double rps = safeRpm / 60.0;
    shooter.setControl(velocityVoltage.withVelocity(rps));
    if (state == ShooterState.IDLE) state = ShooterState.SPINNING_UP;
  }

  public void runAtTargetVelocityPID() {
    runVelocityPIDRpm(targetRpm);
  }

  // ---------------- Telemetry ----------------

  public double getVelocityRpm() {
    double rps = shooter.getVelocity().getValueAsDouble();
    return rps * 60.0;
  }

  /** Estimated acceleration (RPM per second), low-pass filtered. */
  public double getAccelerationRpmPerSec() {
    return accelRpmPerSec;
  }

  public ShooterState getState() {
    return state;
  }

  public boolean isReady() {
    return ready;
  }

  public int getShotCount() {
    return shotCount;
  }

  public double getLastDipMagnitudeRpm() {
    return lastDipMagnitudeRpm;
  }

  public double getLastRecoveryTimeS() {
    return lastRecoveryTimeS;
  }

  // ---------------- State machine ----------------

  @Override
  public void periodic() {
    final double dt = 0.02;

    double rpm = getVelocityRpm();
    double target = targetRpm;
    double err = target - rpm;

    // Accel estimate
    double rawAccel = (rpm - lastVelRpm) / dt;
    accelRpmPerSec = accelRpmPerSec + Constants.Shooter.ACCEL_ALPHA * (rawAccel - accelRpmPerSec);
    lastVelRpm = rpm;

    boolean withinTol = Math.abs(err) <= Constants.Shooter.READY_RPM_TOLERANCE;

    switch (state) {
      case IDLE -> {
        ready = false;
        stableTime = 0.0;
      }

      case SPINNING_UP -> {
        ready = false;
        if (withinTol) stableTime += dt;
        else stableTime = 0.0;

        if (stableTime >= Constants.Shooter.READY_STABLE_TIME_S) {
          state = ShooterState.AT_SPEED;
          ready = true;
          stableTime = 0.0;
        }
      }

      case AT_SPEED -> {
        ready = true;
        if (rpm < target - Constants.Shooter.DIP_DETECT_RPM) {
          state = ShooterState.DIP_DETECTED;
          dipTime = 0.0;
          lastDipMinRpm = rpm;
          lastDipMagnitudeRpm = target - rpm;
        }
      }

      case DIP_DETECTED -> {
        ready = false;
        dipTime += dt;

        if (rpm < lastDipMinRpm) {
          lastDipMinRpm = rpm;
          lastDipMagnitudeRpm = target - rpm;
        }

        if (dipTime >= Constants.Shooter.DIP_MIN_TIME_S) {
          state = ShooterState.RECOVERING;
          recoverStableTime = 0.0;
          lastRecoveryTimeS = 0.0;
        }
      }

      case RECOVERING -> {
        ready = false;
        lastRecoveryTimeS += dt;

        if (withinTol) recoverStableTime += dt;
        else recoverStableTime = 0.0;

        if (recoverStableTime >= Constants.Shooter.RECOVER_STABLE_TIME_S) {
          shotCount++;
          state = ShooterState.AT_SPEED;
          ready = true;
        }
      }
    }

    // NT logging (AdvantageScope-friendly)
    SmartDashboard.putNumber("Shooter/TargetRPM", target);
    SmartDashboard.putNumber("Shooter/ActualRPM", rpm);
    SmartDashboard.putNumber("Shooter/AccelRPMperSec", accelRpmPerSec);
    SmartDashboard.putString("Shooter/State", state.name());
    SmartDashboard.putBoolean("Shooter/Ready", ready);
    SmartDashboard.putNumber("Shooter/DipMagnitudeRPM", lastDipMagnitudeRpm);
    SmartDashboard.putNumber("Shooter/RecoveryTimeS", lastRecoveryTimeS);
    SmartDashboard.putNumber("Shooter/ShotCount", shotCount);
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  @Override
public void simulationPeriodic() {
  var sim = shooter.getSimState();

  double batteryV = RoboRioSim.getVInVoltage();
  sim.setSupplyVoltage(batteryV);

  // CTRE provides the motor voltage being applied; feed it into the physics sim
  double motorVolts = sim.getMotorVoltage();
  flywheelSim.setInputVoltage(motorVolts);
  flywheelSim.update(0.02);

  // Convert flywheel state -> rotor state
  double rotorRps = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

  // Integrate position ourselves (rotations)
  simRotorPositionRot += rotorRps * 0.02;

  sim.setRotorVelocity(rotorRps);
  sim.setRawRotorPosition(simRotorPositionRot);

  // Battery sag
  RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps())
  );
}

}
