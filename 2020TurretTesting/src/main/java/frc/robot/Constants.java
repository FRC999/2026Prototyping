// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

 public static final class Turret {
    public static final int TALON_SRX_CAN_ID = 31;

    public static final int ABS_FORWARD_TICKS = 2900;
    public static final int ABS_TICKS_PER_REV = 4096;

    public static final double REL_TICKS_PER_DEG = 100.0;

    public static final boolean MOTOR_INVERTED = false; // you will tune if needed
    public static final boolean SENSOR_PHASE = false;   // you will tune if needed

    public static final int CONTINUOUS_CURRENT_LIMIT_A = 30;
    public static final int PEAK_CURRENT_LIMIT_A = 45;
    public static final int PEAK_CURRENT_DURATION_MS = 200;

     /** Sim assumptions (tune later). */
    public static final double SIM_GEAR_RATIO = 1.0;
    public static final double SIM_TURRET_J_KGM2 = 0.002; // placeholder; you can tune after measurement

    // For acceleration estimation
    public static final double ACCEL_ALPHA = 0.2; // low-pass filter (0..1)

        /**
     * SAFE ANGLE WINDOW (placeholder).
     * You MUST adjust after measuring your true safe range to protect umbilical.
     *
     * Example: if you have ~380° total, you might allow something like [-190, +190].
     */
    public static final double MIN_ANGLE_DEG = -170.0;
    public static final double MAX_ANGLE_DEG = +170.0;

    /** Turret max output clamp for safety (percent output). */
    public static final double MAX_OUTPUT = 0.35;

    /** Peak voltage clamp if you use voltage control-like behavior. */
    public static final double MAX_VOLTS_EQUIV = 6.0; // used as a conceptual clamp for safety

    /** PID constants (placeholder). Tune on the real turret. */
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 10.0;
    public static final double kF = 0.0;

    /** Motion Magic (optional) placeholders. Units are in sensor units. */
    public static final int MM_CRUISE_VEL = 1500;      // ticks / 100ms
    public static final int MM_ACCEL      = 3000;      // ticks / 100ms / s

    /** Turret location relative to robot center (meters). */
    public static final double DX_METERS = Units.inchesToMeters(6.0); // placeholder
    public static final double DY_METERS = Units.inchesToMeters(0.0); // placeholder

    /** Acceptable aim error for "at setpoint" (degrees). */
    public static final double AT_SETPOINT_DEG = 1.0;

  }

  public static final class Shooter {
    public static final int KRAKEN_CAN_ID = 30;

    // Phoenix 6 prefers inversion/neutral via config enums:
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive; 
    // NOTE: If this runs the wrong reminder direction, flip to CounterClockwise_Positive.
    // You said "negative expels ball" in duty-cycle terms last year. In Phoenix 6,
    // "which direction is positive" is defined by this enum, not a boolean.

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake; // per your preference

    public static final double RPM_STEP = 10.0;
    public static final double DEFAULT_RPM = 3000.0;

    public static final double MAX_RPM = 4800.0; // you estimated with 0.8 duty; feel free to change
    public static final double MAX_DUTY_CYCLE = 0.8;
    public static final double MAX_VOLTS = 12.0;

    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final double SUPPLY_CURRENT_LIMIT_A = 60.0;
    public static final double SUPPLY_CURRENT_TRIGGER_A = 65.0;
    public static final double SUPPLY_CURRENT_TRIGGER_TIME_S = 0.2;

    public static final double STATOR_CURRENT_LIMIT_A = 60.0;

    // Lower-limit behavior (set time = 0 to disable)
    public static final double SUPPLY_CURRENT_LOWER_TIME_S = 0.0;   // disable initially
    public static final double SUPPLY_CURRENT_LOWER_LIMIT_A = 40.0; // only used if time > 0


    // PID placeholders
    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    // Feedforward terms (populate from SysId)
    public static final double kS = 0.0;   // volts
    public static final double kV = 0.12;  // volts per (rot/s)
    public static final double kA = 0.0;   // volts per (rot/s^2)

    public static final double READY_RPM_TOLERANCE = 75.0;
    public static final double READY_STABLE_TIME_S = 0.25;

    public static final double DIP_DETECT_RPM = 250.0;
    public static final double DIP_MIN_TIME_S = 0.04;
    public static final double RECOVER_STABLE_TIME_S = 0.20;

    public static final double ACCEL_ALPHA = 0.2; // low-pass filter

    
    /** Sim inertia (computed below suggestion; placeholder). */
    public static final double SIM_J_KGM2 = 0.010; // placeholder
  }


  public static final class SysId {

    /** 
     * MUST be true to allow any characterization commands to run.
     * Leave FALSE for normal driving.
     */
    public static final boolean ENABLE_SYSID = false;


    // Keep these conservative for a turret + prototype shooter.
    // You can increase once you verify everything is safe.
    public static final double SHOOTER_RAMP_RATE_V_PER_S = 1.0;   // quasistatic
    public static final double SHOOTER_STEP_V = 6.0;              // dynamic
    public static final double SHOOTER_TIMEOUT_S = 8.0;

    public static final double TURRET_RAMP_RATE_V_PER_S = 0.5;    // slower = safer
    public static final double TURRET_STEP_V = 3.0;
    public static final double TURRET_TIMEOUT_S = 6.0;
  }

  public static final class Logging {
    public static final boolean ENABLE_ADVANTAGE_LOGGING = true;
  }

  public static final class OI {
    public static final int DRIVER_JOYSTICK_PORT = 0;

    public static final int BTN_SHOOTER_START = 1;
    public static final int BTN_SHOOTER_STOP  = 2;

    public static final int BTN_RPM_UP   = 3;
    public static final int BTN_RPM_DOWN = 4;

    public static final int BTN_TURRET_JOG_LEFT  = 5;
    public static final int BTN_TURRET_JOG_RIGHT = 6;

    public static final int BTN_TURRET_ZERO = 7;

    public static final int BTN_TURRET_GOTO_0_DEG = 8;
    public static final int BTN_TURRET_GOTO_45_DEG = 9;

    // SysId (hold buttons). If you prefer, use the SmartDashboard buttons instead.
    public static final int BTN_SYSID_SHOOTER_QUASI_FWD = 10;
    public static final int BTN_SYSID_SHOOTER_QUASI_REV = 11;
    public static final int BTN_SYSID_SHOOTER_DYN_FWD   = 12;
    // Shooter dynamic reverse and turret SysId are exposed on SmartDashboard.
  }




}
