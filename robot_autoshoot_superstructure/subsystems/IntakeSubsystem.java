// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.MotionMagicDutyCycleConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.PositionDutyCycleConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeRollerMotor;
  private TalonFX intakePivotMotor;

  private MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0); // for MotionMagic Duty Cycle
  private double intakePivotEncoderZero = 0;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeRollerMotor = new TalonFX(IntakeConstants.intakeRollerMotorId);
    intakePivotMotor = new TalonFX(IntakeConstants.intakePivotMotorId);

    configureMotors();
  }

  private void configureMotors() {
    intakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeRollerMotor.setSafetyEnabled(false);

    var motorRollerConfig = new MotorOutputConfigs();
    motorRollerConfig.NeutralMode = NeutralModeValue.Brake;
    motorRollerConfig.Inverted = (IntakeConstants.IntakeRollerInverted ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
     var talonFXRollerConfigurator = intakeRollerMotor.getConfigurator();

    TalonFXConfiguration pidRollerConfig = new TalonFXConfiguration().withMotorOutput(motorRollerConfig);
    
     StatusCode statusRoller = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusRoller = talonFXRollerConfigurator.apply(pidRollerConfig);
      if (statusRoller.isOK())
        break;
    }
    if (!statusRoller.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusRoller.toString());
    }

     intakePivotMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakePivotMotor.setSafetyEnabled(false);

    var motorPivotConfig = new MotorOutputConfigs();
    motorPivotConfig.NeutralMode = NeutralModeValue.Brake;
    motorPivotConfig.Inverted = (IntakeConstants.IntakeRollerInverted ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
     var talonFXPivotConfigurator = intakeRollerMotor.getConfigurator();

    TalonFXConfiguration pidPivotConfig = new TalonFXConfiguration().withMotorOutput(motorRollerConfig);
    configureMotionMagicDutyCycle(pidPivotConfig);
     StatusCode statusPivot = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusPivot = talonFXPivotConfigurator.apply(pidRollerConfig);
      if (statusPivot.isOK())
        break;
    }
    if (!statusPivot.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusPivot.toString());
    }
  }

  private void configurePositionDutyCycle(TalonFXConfiguration config) {
    config.Slot0.kV = PositionDutyCycleConstants.intake_kV;
    config.Slot0.kP = PositionDutyCycleConstants.intake_kP;
    config.Slot0.kI = PositionDutyCycleConstants.intake_kI;
    config.Slot0.kD = PositionDutyCycleConstants.intake_kD;
  }

  private void setPositionDutyCycle(double position){
    intakePivotMotor.setControl(new PositionDutyCycle(position));
  }

  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
    //config.Slot0.kS = 0.24; // add 0.24 V to overcome friction
    //config.Slot0.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = MotionMagicDutyCycleConstants.intake_kP;
    config.Slot0.kI = MotionMagicDutyCycleConstants.intake_kI;
    config.Slot0.kD = MotionMagicDutyCycleConstants.intake_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicDutyCycleConstants.motionMagicJerk;

    motMagDutyCycle.Slot = MotionMagicDutyCycleConstants.slot;
  }

  public void setMotionMagicDutyCycle(double position){
    intakePivotMotor.setControl(motMagDutyCycle.withPosition(position));
    System.out.println("***Pos: " + position);
  }

  /**
   * Run intake motor at the specified speed
   * 
   * @param speed
   */
  public void runIntake(double speed) {
    intakeRollerMotor.set(speed);
  }

  /**
   * Stop rotating the intake
   */
  public void stopIntake() {
    intakeRollerMotor.set(0);
  }

  public double getIntakePivotEncoderZeroPosition() {
    return intakePivotEncoderZero;
  }

  public double getIntakePivotMotorEncoder() {
    return intakePivotMotor.getRotorPosition().getValueAsDouble();
  }

  public void setIntakePositionWithAngle(IntakePositions angle) { 
    setMotionMagicDutyCycle(intakePivotEncoderZero + angle.getPosition());
  }

  public boolean isAtPosition(IntakePositions position){
    // System.out.println("ME: " + getKrakenMotorEncoder() + " P: " + position.getPosition());
    return Math.abs(position.getPosition() - getIntakePivotMotorEncoder())<=IntakePidConstants.tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
