// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.DriveSubsystem;
import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

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
		public static final boolean odometry = true;
		public static final boolean imu = true;
		public static final boolean chassis = false;
		public static final boolean ll = true;
		public static final boolean questnav = true;
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
  
      public static enum ControllerDevice {
        XBOX_CONTROLLER(
            5, // Port Number for Xbox controller
            ControllerDeviceType.XBOX,
            0.03, // deadband X for Xbox
            0.05, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
            0.03, // deadband Omega for Xbox
            false, // No cube controller configuration for Xbox yet
            false);
  
        private ControllerDeviceType controllerDeviceType;
        private int portNumber;
        private double deadbandX;
        private double deadbandY;
        private double deadbandOmega;
        private boolean cubeControllerLeftStick;
        private boolean cubeControllerRightStick;
  
        ControllerDevice(int pn, ControllerDeviceType cdt, double dx, double dy, double dm, boolean ccL,
            boolean ccR) {
          this.portNumber = pn;
          this.controllerDeviceType = cdt;
          this.deadbandX = dx;
          this.deadbandY = dy;
          this.deadbandOmega = dm;
          this.cubeControllerLeftStick = ccL;
          this.cubeControllerRightStick = ccR;
        }
  
        public ControllerDeviceType getControllerDeviceType() {
          return controllerDeviceType;
        }
  
        public int getPortNumber() {
          return portNumber;
        }
  
        public double getDeadbandX() {
          return deadbandX;
        }
  
        public double getDeadbandY() {
          return deadbandY;
        }
  
        public double getDeadbandOmega() {
          return deadbandOmega;
        }
  
        public boolean isCubeControllerLeftStick() {
          return cubeControllerLeftStick;
        }
  
        public boolean isCubeControllerRightStick() {
          return cubeControllerRightStick;
        }
      }
    }
    /** Swerve-wide constants and module mappings */
    public static final class SwerveConstants {

      public static final double CHASSIS_POSE_HISTORY_TIME = 0.6; //seconds

      public static final boolean CTR_ODOMETRY_UPDATE_FROM_QUEST = true;

      public static final double MaxSpeed = 5.21; // m/s
      public static final double MaxAngularRate = 4.71238898038469; // rad/s
      public static final double DeadbandRatioLinear = 0.05; //determined by calibration method 
      public static final double DeadbandRatioAngular =  0.05; //determined by calibration method

      public static final CANBus kCANBus = new CANBus("canivore1", "./logs/example.hoot");
      public static final Pigeon2Configuration pigeonConfigs = null;
      public static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(100).withKI(0).withKD(0.5)
          .withKS(0.1).withKV(2.66).withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
      public static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(0.1).withKI(0).withKD(0)
          .withKS(0).withKV(0.124);
      public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
      public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
      public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

      // Added from original TunerConstants (auto-merged):

      // Auto-merged constant declarations from original TunerConstants:
      public static final double kCoupleRatio = 3.5714285714285716;
      public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
      public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
      public static final double kDriveGearRatio = 6.122448979591837 * (1.00/0.9);
      public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
      public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
      public static final int kPigeonId = 40;
      public static final Current kSlipCurrent = Amps.of(120.0);
      public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.21);
      public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
      public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
      public static final double kSteerGearRatio = 21.428571428571427;
      public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
      public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
      public static final Distance kWheelRadius = Inches.of(2);

      public static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
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

      /** Module wiring and offsets: MOD0..MOD3 */
      public static enum SwerveModuleConstantsEnum {
        MOD0(3, 4, 31, -0.282470578125, false, true, false), 
        MOD1(1, 2, 30, 0.029541015625, true, true, false),
        MOD2(7, 8, 33, 0.317138875, false, true, false), 
        MOD3(5, 6, 32, 0.044677734375, true, true, false);

        private final int driveMotorID;
        private final int angleMotorID;
        private final int cancoderID;
        private final double angleOffset;
        private final boolean driveMotorInverted;
        private final boolean angleMotorInverted;
        private final boolean cancoderInverted;

        SwerveModuleConstantsEnum(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset,
            boolean driveMotorInverted, boolean angleMotorInverted, boolean cancoderInverted) {
          this.driveMotorID = driveMotorID;
          this.angleMotorID = angleMotorID;
          this.cancoderID = cancoderID;
          this.angleOffset = angleOffset;
          this.driveMotorInverted = driveMotorInverted;
          this.angleMotorInverted = angleMotorInverted;
          this.cancoderInverted = cancoderInverted;
        }

        public int getDriveMotorID() {
          return driveMotorID;
        }

        public int getAngleMotorID() {
          return angleMotorID;
        }

        public int getCancoderID() {
          return cancoderID;
        }

        public double getAngleOffset() {
          return angleOffset;
        }

        public boolean isDriveMotorInverted() {
          return driveMotorInverted;
        }

        public boolean isAngleMotorInverted() {
          return angleMotorInverted;
        }

        public boolean isCancoderInverted() {
          return cancoderInverted;
        }
      }
    }

  }

  public static final class PathPlannerConstants{
    public static final boolean shouldFlipTrajectoryOnRed = true;
  }

  

}