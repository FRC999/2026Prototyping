// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BobsChassis;
import frc.robot.RobotContainer;

public class DriveSubSystem extends SubsystemBase {
  /** Creates a new DriveSubSystem. */
  public TalonFX leftFX;
  public TalonFXSimState leftFXSim;
  // public CANcoder leftEncoder;
  // public CANcoderSimState leftEncoderSim;
  public DCMotorSim dcSim;

  public DriveSubSystem() {
    leftFX = new TalonFX(1);
    leftFXSim = leftFX.getSimState();
    // leftEncoder = new CANcoder(1);
    // leftEncoderSim = leftEncoder.getSimState();
    dcSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, BobsChassis.kGearRatio),
        DCMotor.getKrakenX60(1));
  }

  public void bob() {
    leftFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    // leftEncoderSim.Orientation = ChassisReference.CounterClockwise_Positive;
  }

  @Override
  public void simulationPeriodic() {
    
    
    leftFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("mV", RobotController.getBatteryVoltage());

    //dcSim.setInputVoltage(RobotContainer.joyingStik.getLeftX());
    //dcSim.update(0.020);
    dcSim.setInput(leftFXSim.getMotorVoltage());
    dcSim.update(0.020);
    
    leftFXSim.setRawRotorPosition(dcSim.getAngularPosition().times(BobsChassis.kGearRatio));
    leftFXSim.setRotorVelocity(dcSim.getAngularVelocity().times(BobsChassis.kGearRatio));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftFX.set(RobotContainer.joyingStik.getLeftX());
  }
}
