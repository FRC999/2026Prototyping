// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class EnabledSubsystems {

		public static final boolean chasis = true;
		public static final boolean ll = true;
		public static final boolean questnav = true;
	}

	public static final class DebugTelemetrySubsystems {
		public static final boolean odometry = false;
		public static final boolean imu = true;
		public static final boolean chassis = true;
		public static final boolean ll = false;
		public static final boolean questnav = false;
	}
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static class OIContants {

      public static enum ControllerDeviceType {
        LOGITECH,
        PS5,
        XBOX, // RightJ F/B, LeftJ L/R, L2/R2 - rotation
        XBOX_ONEDRIVE // RIghtJ F/B/L/R, LeftJ - rotation
      }



      public static record ControllerDevice(int portNumber, ControllerDeviceType controllerDeviceType, 
              double deadbandX, double deadbandY, double deadbandOmega, 
              boolean cubeControllerLeftStick, boolean cubeControllerRightStick) {}

      public static ControllerDevice XBOX_CONTROLLER = new ControllerDevice(
        5, 
        ControllerDeviceType.XBOX, 
        0.03, 
        0.05, 
        0.03, 
        false, 
        false);

   
    }
    /** Swerve-wide constants and module mappings */
    public static final class SwerveConstants {

      public static final Distance kXPos = Inches.of(11.75);
      public static final Distance kYPos = Inches.of(11.75);

      public static final double CHASSIS_POSE_HISTORY_TIME = 0.6; //seconds

      public static final boolean CTR_ODOMETRY_UPDATE_FROM_QUEST = true;

      public static final double MaxSpeed = 5.21; // m/s
      public static final double MaxAngularRate = 4.71238898038469; // rad/s
      public static final double DeadbandRatioLinear = 0.05; //determined by calibration method 
      public static final double DeadbandRatioAngular =  0.05; //determined by calibration method

      //public static final CANBus kCANBus = new CANBus("canivore1", "./logs/example.hoot"); // 2025
      public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot"); // 2024 no canivore

      public static final Pigeon2Configuration pigeonConfigs = null;
      public static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(40).withKI(0).withKD(0.5)
          .withKS(0.1).withKV(0.58).withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
      public static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(0.1).withKI(0).withKD(0)
          .withKS(0).withKV(0.122);
      public static final TalonFXSConfiguration steerInitialConfigs = new TalonFXSConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(40))
                  .withStatorCurrentLimitEnable(true));
      public static final TalonFXSConfiguration driveInitialConfigs = new TalonFXSConfiguration();
      public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

      // Added from original TunerConstants (auto-merged):

      // Auto-merged constant declarations from original TunerConstants:
      public static final double kCoupleRatio = 2.8333333333333335;
      public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
      public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
      public static final double kDriveGearRatio =  6.538461538461539;
      public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
      public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFXS_NEO_JST;
      
      public static final int kPigeonId = 15; // 
      //public static final int kPigeonId = 15; // 2024
      
      public static final Current kSlipCurrent = Amps.of(80.0);
      public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.78);
      public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.TalonFXS_PulseWidth;
      public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
      public static final double kSteerGearRatio = 15.42857142857143;
      public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
      public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFXS_Brushed_AB;
      public static final Distance kWheelRadius = Inches.of(2);

      public static SwerveModuleConstantsFactory<TalonFXSConfiguration, TalonFXSConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXSConfiguration, TalonFXSConfiguration, CANcoderConfiguration>()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCoupleRatio)
          .withWheelRadius(kWheelRadius)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
          .withSlipCurrent(kSlipCurrent)
          .withSpeedAt12Volts(kSpeedAt12Volts)
          .withDriveMotorType(kDriveMotorType)
          .withSteerMotorType(kSteerMotorType)
          .withFeedbackSource(kSteerFeedbackType)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withEncoderInitialConfigs(encoderInitialConfigs)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage);
      public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

      public static record SwerveModuleConstantsRecord(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset,
        boolean driveMotorInverted, boolean angleMotorInverted, boolean cancoderInverted) {}


        // 2024 SWERVE CONSTANTS
        
        /*public static final SwerveModuleConstantsRecord MOD0 = new SwerveModuleConstantsRecord(
          1, 
          2, 
          20, 
          -0.282470578125, 
          false, 
          true, 
          false);

        public static final SwerveModuleConstantsRecord MOD1 = new SwerveModuleConstantsRecord(
          3, 
          4, 
          21, 
          0.029541015625, 
          true, 
          true, 
          false);

        public static final SwerveModuleConstantsRecord MOD2 = new SwerveModuleConstantsRecord(
          5, 
          6, 
          22, 
          0.317138875, 
          false, 
          true, 
          false);

        public static final SwerveModuleConstantsRecord MOD3 = new SwerveModuleConstantsRecord(
          7, 
          8, 
          23, 
          0.044677734375, 
          true, 
          true, 
          false); 
        */

          // 2025 Constants
          
          public static final SwerveModuleConstantsRecord MOD0 = new SwerveModuleConstantsRecord( // Front Left,
						8, // driveMotorID
						7, // angleMotorID
						7, // CanCoder Id
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						-0.33837890625 + 0.5 - 0.025391 - 0.5, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);
       
        public static final SwerveModuleConstantsRecord MOD1 = new SwerveModuleConstantsRecord( // Front Right
						2, // driveMotorID
						1, // angleMotorID
						1, // CanCoder Id
						// 0.041015625, // angleOffset of cancoder to mark zero-position
						-0.361328125 + 0.013 - 0.5, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);

        public static final SwerveModuleConstantsRecord MOD2 = new SwerveModuleConstantsRecord( // Back Left
						6, // driveMotorID
						5, // angleMotorID
						5, // CanCoder Id
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						0.742431640625 + 0.025391 - 0.5, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);
        public static final SwerveModuleConstantsRecord MOD3 = new SwerveModuleConstantsRecord( // Back Right
						4, // driveMotorID
						3, // angleMotorID
						3, // CanCoder Id
						// 0.326171875, // angleOffset of cancoder to mark zero-position
						//0.0576171875, // angleOffset of cancoder to mark zero-position
						-0.298828125 -0.5, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);


    }

    /** Turret prototype (Talon FXS + CTRE Mag encoder via gadgeteer port). */
    public static final class Turret {
      public static final int CAN_ID = 21;

      /** Use the same CAN bus as the drivetrain by default. */
      public static final String CANBUS_NAME = OperatorConstants.SwerveConstants.kCANBus.getName();

      /** CTRE Mag absolute reference (PulseWidth 0-4095 equivalent). */
      public static final int ABS_TICKS_PER_REV = 4096;
      /** Absolute encoder tick value that corresponds to turret pointing forward. */
      public static final int ABS_FORWARD_TICKS = 3659;

      /** Mechanical safe range relative to forward (degrees). */
      public static final double MIN_ANGLE_DEG = -190.0;
      public static final double MAX_ANGLE_DEG = 190.0;
      /**
       * When within this margin of a limit, prefer turning the other direction when
       * possible.
       */
      public static final double LIMIT_MARGIN_DEG = 10.0;

      /**
       * Control direction conventions: CCW is positive. Start with false; adjust as
       * needed on-robot.
       */
      public static final boolean MOTOR_INVERTED = false;

      /** Output safety limits. */
      public static final double MAX_DUTY_CYCLE = 0.8;
      public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;
      public static final double STATOR_CURRENT_LIMIT_A = 40.0;

      /** Placeholder gains (Position control). Tune after SysId. */
      public static final double kP = 40.0;
      public static final double kI = 0.0;
      public static final double kD = 2.0;
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      /** MotionMagic placeholders (rotations-based). */
      public static final double MM_CRUISE_VEL_RPS = 1.0;
      public static final double MM_ACCEL_RPS2 = 2.0;

      /** Simulation placeholders. */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_TURRET_J_KGM2 = 0.02;
    }

    /** Shooter prototype (Kraken X60 on TalonFX, Phoenix 6). */
    public static final class Shooter {
      public static final int CAN_ID = 20;
      public static final String CANBUS_NAME = OperatorConstants.SwerveConstants.kCANBus.getName();

      /** Shooter expels ball on negative output, so invert motor. */
      public static final boolean MOTOR_INVERTED = true;
      public static final boolean NEUTRAL_COAST = true;

      public static final double MAX_DUTY_CYCLE = 0.8;
      public static final double SUPPLY_CURRENT_LIMIT_A = 60.0;
      public static final double STATOR_CURRENT_LIMIT_A = 60.0;

      /** Velocity control gains (placeholders). */
      public static final double kP = 0.15;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      /** Setpoint logic. */
      public static final double DEFAULT_RPM = 3000.0;
      public static final double RPM_STEP = 10.0;
      public static final double READY_TOLERANCE_RPM = 50.0;
      public static final double READY_MIN_TIME_S = 0.20;
      public static final double DIP_DETECT_DROP_RPM = 250.0;

      /** Simulation placeholders (combined wheel+flywheel inertia). */
      /** Simulation: motor rotations / wheel rotations. 1.0 for your 1:1 belt. */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_J_KGM2 = 0.02;
    }

    /** SysId gating + default parameters. */
    public static final class SysId {
      /** Safety gate: characterization only runs if true. */
      public static final boolean ENABLE_SYSID = false;

      /**
       * Runtime gate (SmartDashboard boolean). Both this and ENABLE_SYSID must be
       * true.
       */
      public static final String SYSID_DASH_ENABLE_KEY = "SysId/Enable";

      public static final double TURRET_RAMP_RATE_V_PER_S = 1.0;
      public static final double TURRET_STEP_V = 4.0;
      public static final double TURRET_TIMEOUT_S = 10.0;

      public static final double SHOOTER_RAMP_RATE_V_PER_S = 1.0;
      public static final double SHOOTER_STEP_V = 4.0;
      public static final double SHOOTER_TIMEOUT_S = 10.0;
    }

  }
  

  public static final class PathPlannerConstants{
    public static final boolean shouldFlipTrajectoryOnRed = true;
  }

  

}