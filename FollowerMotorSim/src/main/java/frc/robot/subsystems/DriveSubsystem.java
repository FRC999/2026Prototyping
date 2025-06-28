// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  DCMotorSim leadDCMotorSim;
  DCMotorSim followerDCMotorSim;

  TalonFX leadMotor;
  TalonFX followMotor;
  TalonFXSimState leaderSimState;
  TalonFXSimState followerSimState;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leadDCMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, MotorConstants.kGearRatio),
        DCMotor.getKrakenX60(1));
    followerDCMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, MotorConstants.kGearRatio),
        DCMotor.getKrakenX60(1));
    leadMotor = new TalonFX(1);
    followMotor = new TalonFX(2);
    configureMotors();

    leaderSimState = leadMotor.getSimState();
    followerSimState = followMotor.getSimState();
  }

  public void configureMotors() {
    // This is the only place that you set the follow motor to follow the lead motor
    leadMotor.getConfigurator().apply(new TalonFXConfiguration());
    followMotor.getConfigurator().apply(new TalonFXConfiguration());
    followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
  }

  public void simulationInit() {
    leaderSimState.Orientation = ChassisReference.Clockwise_Positive;
    followerSimState.Orientation = ChassisReference.CounterClockwise_Positive;
  }

  @Override
  public void simulationPeriodic() {
    leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = leaderSimState.getMotorVoltageMeasure();
    SmartDashboard.putNumber("mV", motorVoltage.in(Volts));
    SmartDashboard.putNumber("bV",RobotController.getBatteryVoltage());

    // leadDCMotorSim.setInputVoltage(motorVoltage.in(Volts)*RobotContainer.xboxController.getLeftX());
    leadDCMotorSim.setInput(leaderSimState.getMotorVoltage());
    leadDCMotorSim.update(0.020);

    // followerDCMotorSim.setInputVoltage(leaderSimState.getMotorVoltage());
    followerDCMotorSim.setInput(followerSimState.getMotorVoltage());
    followerDCMotorSim.update(0.020);

    leaderSimState.setRawRotorPosition(leadDCMotorSim.getAngularPosition().times(MotorConstants.kGearRatio));
    leaderSimState.setRotorVelocity(leadDCMotorSim.getAngularVelocity().times(MotorConstants.kGearRatio));
    followerSimState.setRawRotorPosition(followerDCMotorSim.getAngularPosition().times(MotorConstants.kGearRatio));
    followerSimState.setRotorVelocity(followerDCMotorSim.getAngularVelocity().times(MotorConstants.kGearRatio));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // This is the only place you have to give the motor power
    /*
     * The code will function as a normal code without simulation,
     * and give the follower the same power as leader
     */
    leadMotor.set(RobotContainer.xboxController.getLeftX());
  }
}
