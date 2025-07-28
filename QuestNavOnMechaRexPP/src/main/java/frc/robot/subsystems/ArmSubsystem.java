// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmMotorConstantsEnum;
import frc.robot.Constants.ArmConstants.ArmPIDConstants;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  // NEO motors connected to Spark Max
  public SparkMax armMotorLeft;
  public SparkMax armMotorRight;
  private SparkMax armMotorLeader;

  private SparkClosedLoopController armPIDControllerLeft;
  private SparkClosedLoopController armPIDControllerRight;

  // We wii use built-in NEO encoders for now
  // They're relative, but we can calibrate them based on second Pigeon on the arm
  private RelativeEncoder armEncoderLeft;
  private RelativeEncoder armEncoderRight;
  private RelativeEncoder armEncoderLeader;

  private Pigeon2 armImu;
  private double armEncoderZero;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    if (!EnabledSubsystems.arm) {
      return;
    }

    armMotorLeft = new SparkMax(ArmMotorConstantsEnum.LEFTMOTOR.getArmMotorID(), MotorType.kBrushless);
    armMotorRight = new SparkMax(ArmMotorConstantsEnum.RIGHTMOTOR.getArmMotorID(), MotorType.kBrushless);

    armPIDControllerLeft = armMotorLeft.getClosedLoopController();
    armPIDControllerRight = armMotorRight.getClosedLoopController();

    // Set Arm encoders
    armEncoderLeft = armMotorLeft.getEncoder();
    armEncoderRight = armMotorRight.getEncoder();

    // Main Motor; should not follow the other motor
    configureArmMotors(armMotorLeft, armEncoderLeft, armPIDControllerLeft, ArmMotorConstantsEnum.LEFTMOTOR, null, IdleMode.kBrake);
    // Follower Motor 
    configureArmMotors(armMotorRight, armEncoderRight, armPIDControllerRight, ArmMotorConstantsEnum.RIGHTMOTOR, armMotorLeft, IdleMode.kBrake);
  
      // ==========================
    // === ARM IMU initialization
    // ==========================
    // This IMU should be attached FLAT to the ARM, with X pointing straight
    // forward.
    // The IMU angle will allow us to calibrate NEO encoders rotating the arm.
    //armImu = new Pigeon2(Arm.PIGEON2_ARM_CAN_ID, "rio");

    calibrateArmEncoderToPitch();

    //setArmFeedForward();

    System.out.println("*** Arm Subsystem Initialized");
  }

  private void configureArmMotors(SparkMax motor, RelativeEncoder encoder, SparkClosedLoopController p,
      ArmMotorConstantsEnum c,
      SparkMax motorToFollow, IdleMode mode) {
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    // motor.restoreFactoryDefaults(); //restores the state of the motor to factory
    // defaults
    motor.clearFaults(); // clears a fault that has occurred since the last time the faults were reset
    sparkMaxConfig.inverted(c.getArmMotorInverted()); // sets motor inverted if getArmMotorInverted() returns true

    sparkMaxConfig.idleMode(mode); // sets motor into brake mode
    // motor.setIdleMode(IdleMode.kCoast);

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR); // sets conversion between NEO
                                                                                     // units to necessary unit for
                                                                                     // positon
    sparkMaxConfig.apply(encoderConfig);

    motor.setCANTimeout(0); // sets up timeout

    sparkMaxConfig.voltageCompensation(ArmConstants.nominalVoltage); // enables voltage compensation for set voltage
                                                                     // [12v]

    if (EnableCurrentLimiter.arm) {
      sparkMaxConfig.smartCurrentLimit(CurrentLimiter.arm); // sets current limit to 40 amps
    }

    sparkMaxConfig.openLoopRampRate(ArmConstants.rampRate); // sets the rate to go from 0 to full throttle on open loop
    sparkMaxConfig.closedLoopRampRate(ArmConstants.rampRate); // sets the rate to go from 0 to full throttle on open
                                                              // loop

    SignalsConfig signalsConfig = new SignalsConfig();

    // sets which motor is the leader and follower; set follower inversion if needed
    if (c.getArmMotorFollower()) {
      sparkMaxConfig.follow(motorToFollow, c.getArmMotorInverted());

      // kstatus0
      signalsConfig.faultsPeriodMs(100);
      signalsConfig.appliedOutputPeriodMs(100);
      signalsConfig.outputCurrentPeriodMs(100);

      // kstatus1
      signalsConfig.motorTemperaturePeriodMs(250);
      signalsConfig.primaryEncoderVelocityPeriodMs(250);

      // kstatus2
      signalsConfig.primaryEncoderPositionPeriodMs(250);

    } else {

      System.out.println("*** Set Arm Leader " + motor.getDeviceId());

      armMotorLeader = motor;
      armEncoderLeader = motor.getEncoder();

      // kstatus1
      signalsConfig.motorTemperaturePeriodMs(50);
      signalsConfig.primaryEncoderVelocityPeriodMs(50);

      // kstatus2
      signalsConfig.primaryEncoderPositionPeriodMs(50);

    }

    // apply signals
    sparkMaxConfig.apply(signalsConfig);

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    // --- PID Setup
    // set the PID sensor to motor encoder for hardware PID
    closedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // set arm PID coefficients - LIFT
    closedLoopConfig.p(ArmPIDConstants.kP);
    closedLoopConfig.i(ArmPIDConstants.kI);
    closedLoopConfig.d(ArmPIDConstants.kD);
    closedLoopConfig.iZone(ArmPIDConstants.Izone);
    // p.setFF(ArmPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    closedLoopConfig.outputRange(-ArmPIDConstants.kMaxOutput, ArmPIDConstants.kMaxOutput);

    // Apply closed loop configuration
    sparkMaxConfig.apply(closedLoopConfig);

    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  private void calibrateArmEncoderToPitch(){
        armEncoderZero = getArmEncoderLeader() -
      (
       (getArmIMUPitch() - 
          ((ArmConstants.USE_PAN_IMU_FOR_CORRECTION) ? RobotContainer.driveSubsystem.getPitch() : 0))  // pan IMU Pitch-based correction for uneven surface
            * ArmConstants.ARM_ENCODER_CHANGE_PER_DEGREE
      );
  }

  public void resetArmEncoderToPitch() {
        armEncoderZero = getArmEncoderLeader() -
      (
       (ArmConstants.ARM_IMU_RESET_ANGLE - 
          ((ArmConstants.USE_PAN_IMU_FOR_CORRECTION) ? RobotContainer.driveSubsystem.getPitch() : 0))  // pan IMU Pitch-based correction for uneven surface
            * ArmConstants.ARM_ENCODER_CHANGE_PER_DEGREE
      );
  }

  /**
   * Get Arm IMU pitch in degrees.
   * IMU is pointed to the FRONT of the robot with the X and left with Y
   * Positive Pitch angle increases when arm is going from front towards the back.
   * 
   * @return
   */
  public double getArmIMUPitch() {
    return -armImu.getPitch().getValueAsDouble();
  }

  /**
   * Get current angle of the arm.
   * Positive Pitch angle increases when arm is going from front towards the back.
   * The pitch (per encoders) is a difference between current encoder value and a
   * "zero degree" encoder value
   * divided by ARM_ENCODER_CHANGE_PER_DEGREE
   * 
   * @return - degrees angle
   */
  public double getArmAngleSI() {
    return (armEncoderLeader.getPosition() - armEncoderZero) ; // /Arm.ARM_ENCODER_CHANGE_PER_DEGREE;
  }

  // Encoder telemetry
  public double getArmEncoderLeft() {
    return armEncoderLeft.getPosition();
  }

  public double getArmEncoderRight() {
    return armEncoderRight.getPosition();
  }

  public double getArmEncoderLeader() {
    return armEncoderLeader.getPosition();
  }

  public void runArmMotors(double power) {
    armMotorLeader.set(power);
  }

  public void stopMotors() {
    armMotorLeader.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
