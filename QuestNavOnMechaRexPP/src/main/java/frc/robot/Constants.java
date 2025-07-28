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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.util.Units;
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
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static class EnabledSubsystems {
		public static final boolean arm = true;
	}

	public static class EnableCurrentLimiter {
		public static final boolean arm = true;
	}

	public static final class CurrentLimiter {
		public static int intake = 0;
		public static int arm = 40;
		public static int shooter = 40;
	}

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
					0.03, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
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

	public static class SwerveConstants {
		public static class TunerConstants {
			public static final double steerGainsKP = 6.25;
			public static final double steerGainsKI = 0.0;
			public static final double steerGainsKD = 0.0;
			public static final double steerGainsKS = 0.2;
			public static final double steerGainsKV = 2.66;
			public static final double steerGainsKA = 0;

			private static final Slot0Configs steerGains = new Slot0Configs()
					.withKP(steerGainsKP).withKI(steerGainsKI).withKD(steerGainsKD)
					.withKS(steerGainsKS).withKV(steerGainsKV).withKA(steerGainsKA)
					.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

			public static final double driveGainsKP = 3.5;
			public static final double driveGainsKI = 0.0;
			public static final double driveGainsKD = 0.02;
			public static final double driveGainsKS = 0;
			public static final double driveGainsKV = 0.124;
			public static final double driveGainsKA = 0;

			private static final Slot0Configs driveGains = new Slot0Configs()
					.withKP(driveGainsKP).withKI(driveGainsKI).withKD(driveGainsKD)
					.withKS(driveGainsKS).withKV(driveGainsKV);

			private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
			private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

			// The type of motor used for the drive motor
			private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
			// The type of motor used for the drive motor
			private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
			// The remote sensor feedback type to use for the steer motors;
			// When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
			private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

			private static final Current slipCurrent = Amps.of(120);
			// Initial configs for the drive and steer motors and the azimuth encoder; these
			// cannot be null.
			// Some configs will be overwritten; check the `with*InitialConfigs()` API
			// documentation.
			private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
			private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									// Swerve azimuth does not require much torque output, so we can set a
									// relatively low
									// stator current limit to help avoid brownouts without impacting performance.
									.withStatorCurrentLimit(Amps.of(60))
									.withStatorCurrentLimitEnable(true));
			private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

			// CAN bus that the devices are located on;
			// All swerve devices must share the same CAN bus
			public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

			// Theoretical free speed (m/s) at 12 V applied output;
			// This needs to be tuned to your individual robot
			public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(10.43);

			// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
			// This may need to be tuned to your individual robot
			private static final double kCoupleRatio = 3.5714285714285716;

			private static final double kDriveGearRatio = 6.122448979591837;
			private static final double kSteerGearRatio = 21.428571428571427;
			private static final Distance wheelRadius = Inches.of(4);

			private static final boolean kInvertLeftSide = false;
			private static final boolean kInvertRightSide = true;

			// These are only used for simulation
			private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
			private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
			// Simulated voltage necessary to overcome friction
			private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
			private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

			public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
					.withCANBusName(kCANBus.getName())
					.withPigeon2Id(IMUConstants.kPigeonId)
					.withPigeon2Configs(IMUConstants.pigeonConfigs);

			public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
					.withDriveMotorGearRatio(kDriveGearRatio)
					.withSteerMotorGearRatio(kSteerGearRatio)
					.withCouplingGearRatio(kCoupleRatio)
					.withWheelRadius(wheelRadius)
					.withSteerMotorGains(steerGains)
					.withDriveMotorGains(driveGains)
					.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
					.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
					.withSlipCurrent(slipCurrent)
					.withSpeedAt12Volts(speedAt12Volts)
					.withFeedbackSource(steerFeedbackType)
					.withDriveMotorInitialConfigs(driveInitialConfigs)
					.withSteerMotorInitialConfigs(steerInitialConfigs)
					.withEncoderInitialConfigs(cancoderInitialConfigs)
					.withSteerInertia(kSteerInertia)
					.withDriveInertia(kDriveInertia)
					.withSteerFrictionVoltage(kSteerFrictionVoltage)
					.withDriveFrictionVoltage(kDriveFrictionVoltage);
		}

		public static class SwerveChassis {
			public static final double TRACK_WIDTH = 0.525; // left to right
			public static final double WHEEL_BASE = 0.525; // front to back
			public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
			public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

			public static final double MaxSpeed = TunerConstants.speedAt12Volts.magnitude(); // kSpeedAt12VoltsMps
																								// desired top speed
			public static final double maxAcceleration = 41.68; // this is Max linear acceleration units: m/s^2
			public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular
																		// velocity
			public static final double maxAngularAcceleration = 37.6992; // this is max angular acceleration units:
																			// rad/s^2
			public static final double robotMass = 56.7; // kg
			public static final double robotInertia = 60.0; // KG*M^2 - for rotation
			public static final double wheelCOF = 1.2; // coefficient of friction for the wheels; colsons on carpet is
														// 1.0

			public static enum SwerveModuleConstantsEnum {
				MOD0(
						1,
						2,
						20,
						-0.306640625,
						false,
						true,
						false),
				MOD1(
						3,
						4,
						21,
						0.06298828125,
						false,
						true,
						false),
				MOD2(
						5,
						6,
						22,
						0.3173828125,
						false,
						true,
						false),
				MOD3(
						7,
						8,
						23,
						0.037353515625,
						false,
						true,
						false);

				private int driveMotorID;
				private int angleMotorID;
				private int cancoderID;
				private double angleOffset;
				private boolean driveMotorInverted;
				private boolean angleMotorInverted;
				private boolean cancoderInverted;

				SwerveModuleConstantsEnum(int d, int a, int c, double o,
						boolean di, boolean ai, boolean ci) {
					this.driveMotorID = d;
					this.angleMotorID = a;
					this.cancoderID = c;
					this.angleOffset = o;
					this.driveMotorInverted = di;
					this.angleMotorInverted = ai;
					this.cancoderInverted = ci;
				}

				public int getDriveMotorID() {
					return driveMotorID;
				}

				public int getAngleMotorID() {
					return angleMotorID;
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

				public boolean isCANCoderIverted() {
					return cancoderInverted;
				}

				public int getCancoderID() {
					return cancoderID;
				}
			}
		}
	}

	public static class ArmConstants {
		public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
		public static final double nominalVoltage = 12.0;
		public static final double rampRate = 0.25;
		public static final boolean USE_PAN_IMU_FOR_CORRECTION = true; // Correct Arm IMU with Pan IMU if game
		// surface is uneven
		public static final double ARM_ENCODER_CHANGE_PER_DEGREE = 3.862568732;

		public static final double ARM_IMU_RESET_ANGLE = -82.0;

		public static enum ArmMotorConstantsEnum {
			LEFTMOTOR( // Front Left - main motor
					32, // CANID
					true, // Inversion
					false // Follower
			),
			RIGHTMOTOR( // Front Left
					31, // CANID
					true, // Inversion
					true // Follower
			);

			private int armMotorID; // CAN ID
			private boolean armMotorInverted;
			private boolean armMotorFollower;

			ArmMotorConstantsEnum(int cid, boolean i, boolean f) {
				this.armMotorID = cid;
				this.armMotorInverted = i;
				this.armMotorFollower = f;
			}

			public int getArmMotorID() {
				return armMotorID;
			}

			public boolean getArmMotorInverted() {
				return armMotorInverted;
			}

			public boolean getArmMotorFollower() {
				return armMotorFollower;
			}
		}

		public static final class ArmPIDConstants {

			public static final double kP = 0.02;
			public static final double kI = 0.000;
			public static final double kD = 2.0;
			public static final double kF = 0;
			public static final double kMaxOutput = 0.6;
			public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
			public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
			public static final int Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for
													// S-Curve (greater value yields greater smoothing).
			public static final double DefaultAcceptableError = 5; // Sensor units
			public static final double Izone = 500;
			public static final double PeakOutput = 0.5; // Closed Loop peak output
			public static final double NeutralDeadband = 0.001;
			public static final int periodMs = 10; // status frame period
			public static final int timeoutMs = 30; // status frame timeout
			public static final int closedLoopPeriod = 1; // 1ms for TalonSRX and locally connected encoder

			public static final double anglePIDTolerance = 0.5; // degree tolerance when rotating arm to angle using
																// PID

		}
	}

	public static class IMUConstants {
		private static final int kPigeonId = 15;
		private static final Pigeon2Configuration pigeonConfigs = null;
	}
}
